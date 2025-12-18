import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import copy
class Process:
def __init__(self, pid, burst_time, arrival_time=0):
self.pid = pid
self.burst_time = burst_time
self.remaining_time = burst_time
self.arrival_time = arrival_time
self.completion_time = 0
self.waiting_time = 0
self.turnaround_time = 0
self.priority_score = 0
def reset(self):
self.remaining_time = self.burst_time
self.completion_time = 0
self.waiting_time = 0
self.turnaround_time = 0
self.priority_score = 0
def __repr__(self):
return f"P{self.pid}(BT={self.burst_time})"
def rr_fixed_quantum(processes, quantum=4):
processes = copy.deepcopy(processes)
current_time = 0
queue = processes.copy()
completed = []
context_switches = 0
while queue:
for process in queue[:]:
if process.remaining_time > 0:
exec_time = min(quantum, process.remaining_time)
process.remaining_time -= exec_time
current_time += exec_time
context_switches += 1
if process.remaining_time == 0:
process.completion_time = current_time
process.turnaround_time = current_time - process.arrival_time
process.waiting_time = process.turnaround_time -
process.burst_time
completed.append(process)
queue.remove(process)
return completed, context_switches
def rr_from_research_paper(processes):
processes = copy.deepcopy(processes)
processes.sort(key=lambda x: x.burst_time)
current_time = 0
queue = processes.copy()
completed = []
context_switches = 0
while queue:
active = [p for p in queue if p.remaining_time > 0]
if not active:
break
burst_times = [p.remaining_time for p in active]
mean_bt = np.mean(burst_times)
min_bt = min(burst_times)
max_bt = max(burst_times)
ct = max_bt + min_bt
time_quantum = int(np.sqrt(mean_bt + ct))
time_quantum = max(1, time_quantum)
for process in active:
exec_time = min(time_quantum, process.remaining_time)
process.remaining_time -= exec_time
current_time += exec_time
context_switches += 1
if process.remaining_time == 0:
process.completion_time = current_time
process.turnaround_time = current_time - process.arrival_time
process.waiting_time = process.turnaround_time - process.burst_time
completed.append(process)
queue.remove(process)
return completed, context_switches
def our_rr_model(processes):
processes = copy.deepcopy(processes)
all_bursts = [p.burst_time for p in processes]
threshold = np.median(all_bursts)
short_queue = [p for p in processes if p.burst_time <= threshold]
long_queue = [p for p in processes if p.burst_time > threshold]
short_queue.sort(key=lambda x: x.burst_time)
long_queue.sort(key=lambda x: x.burst_time)
completed = []
current_time = 0
context_switches = 0
def calculate_queue_quantum(queue, factor=1.0):
if not queue:
return 1
remaining = [p.remaining_time for p in queue if p.remaining_time > 0]
if not remaining:
return 1
mean_r = np.mean(remaining)
median_r = np.median(remaining)
return max(1, int(factor * (0.6 * median_r + 0.4 * mean_r)))
while short_queue or long_queue:
for _ in range(2):
if short_queue:
active_short = [p for p in short_queue if p.remaining_time > 0]
if active_short:
tq_short = calculate_queue_quantum(active_short, 0.5)
process = active_short[0]
exec_time = min(tq_short, process.remaining_time)
process.remaining_time -= exec_time
current_time += exec_time
context_switches += 1
if process.remaining_time == 0:
process.completion_time = current_time
process.turnaround_time = current_time -
process.arrival_time
process.waiting_time = process.turnaround_time -
process.burst_time
completed.append(process)
short_queue.remove(process)
if long_queue:
active_long = [p for p in long_queue if p.remaining_time > 0]
if active_long:
tq_long = calculate_queue_quantum(active_long, 1.2)
process = active_long[0]
exec_time = min(tq_long, process.remaining_time)
process.remaining_time -= exec_time
current_time += exec_time
context_switches += 1
if process.remaining_time == 0:
process.completion_time = current_time
process.turnaround_time = current_time - process.arrival_time
process.waiting_time = process.turnaround_time -
process.burst_time
completed.append(process)
long_queue.remove(process)
return completed, context_switches
def calculate_metrics(completed_processes):
avg_waiting_time = np.mean([p.waiting_time for p in completed_processes])
avg_turnaround_time = np.mean([p.turnaround_time for p in completed_processes])
total_burst_time = sum([p.burst_time for p in completed_processes])
total_completion_time = max([p.completion_time for p in completed_processes])
cpu_utilization = (total_burst_time / total_completion_time) * 100 if
total_completion_time > 0 else 0
return {
'avg_waiting_time': avg_waiting_time,
'avg_turnaround_time': avg_turnaround_time,
'cpu_utilization': cpu_utilization
}
def plot_comparison(results, title="Algorithm Comparison"):
algorithms = ['RR with fixed quantum', 'RR from research paper', 'Our RR
model']
fig, axes = plt.subplots(2, 2, figsize=(15, 10))
fig.suptitle(title, fontsize=16, fontweight='bold')
awt_values = [results[alg]['metrics']['avg_waiting_time'] for alg in
algorithms]
colors = ['#FF6B6B', '#4ECDC4', '#45B7D1']
bars1 = axes[0, 0].bar(algorithms, awt_values, color=colors)
axes[0, 0].set_ylabel('Time (ms)', fontsize=11)
axes[0, 0].set_title('Average Waiting Time (Lower is Better)', fontsize=12,
fontweight='bold')
axes[0, 0].grid(axis='y', alpha=0.3)
axes[0, 0].tick_params(axis='x', rotation=15, labelsize=9)
for i, v in enumerate(awt_values):
axes[0, 0].text(i, v + max(awt_values)*0.02, f'{v:.1f}', ha='center',
va='bottom', fontsize=10, fontweight='bold')
att_values = [results[alg]['metrics']['avg_turnaround_time'] for alg in
algorithms]
bars2 = axes[0, 1].bar(algorithms, att_values, color=colors)
axes[0, 1].set_ylabel('Time (ms)', fontsize=11)
axes[0, 1].set_title('Average Turnaround Time (Lower is Better)', fontsize=12,
fontweight='bold')
axes[0, 1].grid(axis='y', alpha=0.3)
axes[0, 1].tick_params(axis='x', rotation=15, labelsize=9)
for i, v in enumerate(att_values):
axes[0, 1].text(i, v + max(att_values)*0.02, f'{v:.1f}', ha='center',
va='bottom', fontsize=10, fontweight='bold')
cs_values = [results[alg]['context_switches'] for alg in algorithms]
bars3 = axes[1, 0].bar(algorithms, cs_values, color=colors)
axes[1, 0].set_ylabel('Number of Switches', fontsize=11)
axes[1, 0].set_title('Context Switches (Lower is Better)', fontsize=12,
fontweight='bold')
axes[1, 0].grid(axis='y', alpha=0.3)
axes[1, 0].tick_params(axis='x', rotation=15, labelsize=9)
for i, v in enumerate(cs_values):
axes[1, 0].text(i, v + max(cs_values)*0.02, f'{v}', ha='center',
va='bottom', fontsize=10, fontweight='bold')
cpu_values = [results[alg]['metrics']['cpu_utilization'] for alg in algorithms]
bars4 = axes[1, 1].bar(algorithms, cpu_values, color=colors)
axes[1, 1].set_ylabel('Utilization (%)', fontsize=11)
axes[1, 1].set_title('CPU Utilization (Higher is Better)', fontsize=12,
fontweight='bold')
axes[1, 1].set_ylim([0, 100])
axes[1, 1].grid(axis='y', alpha=0.3)
axes[1, 1].tick_params(axis='x', rotation=15, labelsize=9)
for i, v in enumerate(cpu_values):
axes[1, 1].text(i, v + 1, f'{v:.1f}%', ha='center', va='bottom',
fontsize=10, fontweight='bold')
plt.tight_layout()
filename = f'{title.replace(" ", "_").replace(":", "").replace("(",
"").replace(")", "")}.png'
plt.savefig(filename, dpi=300, bbox_inches='tight')
plt.close()
def print_results_table(results):
algorithms = ['RR with fixed quantum', 'RR from research paper', 'Our RR
model']
print("\n")
print("RESULTS")
print("-" * 100)
for alg_name in algorithms:
data = results[alg_name]
metrics = data['metrics']
cs = data['context_switches']
print(f"\n{alg_name}:")
print(f" Average Waiting Time: {metrics['avg_waiting_time']:.2f} ms")
print(f" Average Turnaround Time: {metrics['avg_turnaround_time']:.2f}
ms")
print(f" Context Switches: {cs}")
print(f" CPU Utilization: {metrics['cpu_utilization']:.2f}%")
print("\n" + "-" * 100)
baseline_wt = results['RR from research paper']['metrics']['avg_waiting_time']
improved_wt = results['Our RR model']['metrics']['avg_waiting_time']
improvement_wt = ((baseline_wt - improved_wt) / baseline_wt) * 100
baseline_tat = results['RR from research paper']['metrics']
['avg_turnaround_time']
improved_tat = results['Our RR model']['metrics']['avg_turnaround_time']
improvement_tat = ((baseline_tat - improved_tat) / baseline_tat) * 100
baseline_cs = results['RR from research paper']['context_switches']
improved_cs = results['Our RR model']['context_switches']
improvement_cs = ((baseline_cs - improved_cs) / baseline_cs) * 100
print("\nIMPROVEMENT (Our RR model vs RR from research paper):")
print(f" Waiting Time Reduction: {improvement_wt:.1f}%")
print(f" Turnaround Time Reduction: {improvement_tat:.1f}%")
print(f" Context Switches Reduction: {improvement_cs:.1f}%")
print("-" * 100)
print("\n")
def create_test_cases():
test_cases = {}
test_cases['Paper Example'] = [Process(1, 27), Process(2, 54), Process(3, 79),
Process(4, 93), Process(5, 140)]
test_cases['Mixed Workload'] = [Process(1, 5), Process(2, 90), Process(3, 12),
Process(4, 85), Process(5, 8), Process(6, 100), Process(7, 15), Process(8, 78),
Process(9, 10), Process(10, 95)]
np.random.seed(42)
test_cases['Large Scale'] = [Process(i, np.random.randint(5, 100)) for i in
range(1, 51)]
return test_cases
def run_comparison(test_case_name, processes):
print("\n" + "=" * 100)
print(f"TEST: {test_case_name}")
print("=" * 100)
print(f"Processes: {len(processes)}")
if len(processes) <= 10:
print(f"Burst Times: {[p.burst_time for p in processes]}")
print()
results = {}
print("Running algorithms...")
completed_rr, cs_rr = rr_fixed_quantum(processes, quantum=4)
results['RR with fixed quantum'] = {'completed': completed_rr, 'metrics':
calculate_metrics(completed_rr), 'context_switches': cs_rr}
print(" [1/3] RR with fixed quantum - Complete")
completed_irrdq, cs_irrdq = rr_from_research_paper(processes)
results['RR from research paper'] = {'completed': completed_irrdq, 'metrics':
calculate_metrics(completed_irrdq), 'context_switches': cs_irrdq}
print(" [2/3] RR from research paper - Complete")
completed_ours, cs_ours = our_rr_model(processes)
results['Our RR model'] = {'completed': completed_ours, 'metrics':
calculate_metrics(completed_ours), 'context_switches': cs_ours}
print(" [3/3] Our RR model - Complete")
print_results_table(results)
print("Generating graph...")
plot_comparison(results, title=f"Comparison: {test_case_name}")
print(f"Graph saved: Comparison_{test_case_name.replace(' ', '_').replace('(',
'').replace(')', '')}.png")
return results
if __name__ == "__main__":
print("\n" + "=" * 100)
print("CPU SCHEDULING ALGORITHM COMPARISON")
print("Comparing: RR with fixed quantum | RR from research paper | Our RR
model")
print("=" * 100)
test_cases = create_test_cases()
all_results = {}
print("\nRunning 3 test cases...\n")
all_results['Paper Example'] = run_comparison('Paper Example',
test_cases['Paper Example'])
all_results['Mixed Workload'] = run_comparison('Mixed Workload',
test_cases['Mixed Workload'])
all_results['Large Scale'] = run_comparison('Large Scale (50 processes)',
test_cases['Large Scale'])
