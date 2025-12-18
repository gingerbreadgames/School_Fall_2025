import os, json, re, time
from typing import List, Dict, Any, Optional

from dotenv import load_dotenv
load_dotenv()


DB_URI = os.getenv("MYSQL_URI") 
EXPECTED_PATH = "expected.jsonl" 
RESULTS_CSV = "results.csv"

# Models you want to test (Ollama)
MODELS = ["gpt-oss:20b", "llama3.1:8b"]
TEMPERATURE = 0
SLEEP_BETWEEN_CALLS_SEC = 0.4
FLOAT_EPS = 1e-6


from sqlalchemy import create_engine, text, inspect
import pandas as pd
from langchain_ollama import ChatOllama
from langchain_core.prompts import ChatPromptTemplate


def reflect_schema_snippet(engine, max_cols_per_table=20) -> str:
    insp = inspect(engine)
    lines = []
    for table in insp.get_table_names():
        cols = insp.get_columns(table)
        col_str = ", ".join([f"{c['name']} {str(c.get('type'))}" for c in cols[:max_cols_per_table]])
        if len(cols) > max_cols_per_table:
            col_str += ", ..."
        lines.append(f"{table}({col_str})")
    return "Schema:\n" + "\n".join(lines)


def load_expected(path: str) -> List[Dict[str, Any]]:
    out = []
    with open(path, "r", encoding="utf-8") as f:
        for i, line in enumerate(f, 1):
            s = line.strip()
            if not s:
                continue
            try:
                out.append(json.loads(s))
            except json.JSONDecodeError as e:
                raise SystemExit(f"Bad JSON on line {i} of {path}: {e}\nLine: {s}") from e
    return out


def only_first_sql_statement(s: str) -> str:
    s = s.strip()
    s = re.sub(r"^```(?:sql)?\s*", "", s, flags=re.IGNORECASE)
    s = re.sub(r"\s*```$", "", s)
    m = re.search(r";", s)
    return s if not m else s[:m.start()+1]


def extract_scalar_value(rows) -> Optional[float]:
    if not rows:
        return None
    row0 = rows[0]
 
    try:
        val = row0[0]
    except Exception:
        return None
    try:
        return float(val)
    except Exception:
        return None


def values_equal(a: Optional[float], b: Optional[float]) -> bool:
    if a is None or b is None:
        return False
    if float(a).is_integer() and float(b).is_integer():
        return int(round(a)) == int(round(b))
    return abs(a - b) <= FLOAT_EPS


def safe_write_csv(df: pd.DataFrame, path: str):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    df.to_csv(path, index=False, encoding="utf-8-sig", lineterminator="\n")


def looks_suspicious(sql: str) -> bool:
    """Simple heuristics to catch common hallucinations like T2.goals etc."""
    bad_bits = [" T2.", " T1.", ".goals ", ".goals,", ".goals)"]
    return any(bit in sql for bit in bad_bits)


def main():
    print("EXPECTED_PATH =", EXPECTED_PATH)
    expected = load_expected(EXPECTED_PATH)
    if not expected:
        raise SystemExit("No items found in expected.jsonl")
    print("Loaded", len(expected), "expected items. Keys:", list(expected[0].keys()))

    engine = create_engine(DB_URI, pool_pre_ping=True)
    schema_snippet = reflect_schema_snippet(engine)
    print(schema_snippet)

    prompt_tmpl = ChatPromptTemplate.from_template(
        """You are an expert MySQL query writer.

Only use these tables and exact column names:
{schema}

Rules:
- Output ONE MySQL SELECT statement only. No prose, no markdown.
- It must return ONE NUMBER (one row, one column).
- Table names: Team, Player, `Match`, Appearance.
- Columns you may use exactly as spelled in the schema above.
- Do NOT invent columns or tables.
- Do NOT use table aliases in the SELECT list.
- End your SQL with a semicolon.

Q: {question}"""
    )

    header = ["qid","prompt","model","generated_sql","generated_value","gold_sql","gold_value","error","is_correct"]
    safe_write_csv(pd.DataFrame(columns=header), RESULTS_CSV)

    all_rows = []

    with engine.connect() as conn:
        for mdl in MODELS:
            if not SELFTEST:
                llm = ChatOllama(model=mdl, temperature=TEMPERATURE)

            for item in expected:
                qid = item["qid"]
                question = item["prompt"]
                gold_sql = item.get("sql")
                gold_scalar, gold_err = None, ""

                # Compute gold value
                if gold_sql:
                    try:
                        res2 = conn.execute(text(gold_sql))
                        gold_rows = list(res2.fetchall())
                        gold_scalar = extract_scalar_value(gold_rows)
                        if gold_scalar is None:
                            gold_err = "gold query returned no scalar"
                    except Exception as e:
                        gold_err = f"{type(e).__name__}: {e}"
                elif item.get("value") is not None:
                    try:
                        gold_scalar = float(item["value"])
                    except Exception as e:
                        gold_err = f"value cast error: {e}"
                else:
                    gold_err = "no gold_sql or value present"

                
                if SELFTEST:
                    gen_sql = gold_sql or "SELECT 0;"
                else:
                    messages = prompt_tmpl.format_messages(schema=schema_snippet, question=question)
                    resp = llm.invoke(messages)
                    raw_sql = (resp.content or "").strip()
                    gen_sql = only_first_sql_statement(raw_sql)

                              if looks_suspicious(gen_sql):
                    gen_sql = "SELECT 0;"  

               
                gen_value, gen_err = None, ""
                try:
                    res = conn.execute(text(gen_sql))
                    rows_db = list(res.fetchall())
                    gen_value = extract_scalar_value(rows_db)
                except Exception as e:
                    gen_err = f"{type(e).__name__}: {e}"

                is_correct = (gen_value is not None and gold_scalar is not None and values_equal(gen_value, gold_scalar))
                err = gen_err or gold_err

                row = {
                    "qid": qid,
                    "prompt": question,
                    "model": mdl,
                    "generated_sql": gen_sql,
                    "generated_value": gen_value,
                    "gold_sql": gold_sql,
                    "gold_value": gold_scalar,
                    "error": err,
                    "is_correct": bool(is_correct),
                }
                all_rows.append(row)

                
                pd.DataFrame([row], columns=header).to_csv(
                    RESULTS_CSV, mode="a", header=False, index=False, encoding="utf-8-sig", lineterminator="\n"
                )

                time.sleep(SLEEP_BETWEEN_CALLS_SEC)

    df_final = pd.DataFrame(all_rows, columns=header)
    safe_write_csv(df_final, RESULTS_CSV)
    print(f"Wrote {RESULTS_CSV} with {len(df_final)} rows.")

if __name__ == "__main__":
    main()
