  CREATE DATABASE soccerhub;
USE soccerhub;

CREATE TABLE Team (
  teamId INT UNSIGNED PRIMARY KEY AUTO_INCREMENT,
  name VARCHAR(80) NOT NULL UNIQUE,
  city VARCHAR(80) NOT NULL,
  stadium VARCHAR(80)
);

CREATE TABLE Player (
  playerId INT UNSIGNED PRIMARY KEY AUTO_INCREMENT,
  fullName VARCHAR(100) NOT NULL,
  teamId INT UNSIGNED,
  position VARCHAR(20),
  birthYear SMALLINT,
  FOREIGN KEY (teamId) REFERENCES Team(teamId)
);

CREATE TABLE `Match` (
  matchId INT UNSIGNED PRIMARY KEY AUTO_INCREMENT,
  homeTeamId INT UNSIGNED NOT NULL,
  awayTeamId INT UNSIGNED NOT NULL,
  matchDate DATE NOT NULL,
  stadium VARCHAR(80),
  homeGoals TINYINT,
  awayGoals TINYINT,
  FOREIGN KEY (homeTeamId) REFERENCES Team(teamId),
  FOREIGN KEY (awayTeamId) REFERENCES Team(teamId)
);

CREATE TABLE Appearance (
  matchId INT UNSIGNED NOT NULL,
  playerId INT UNSIGNED NOT NULL,
  teamId INT UNSIGNED NOT NULL,
  minutes SMALLINT,
  goals TINYINT,
  assists TINYINT,
  PRIMARY KEY (matchId, playerId),
  FOREIGN KEY (matchId) REFERENCES `Match`(matchId),
  FOREIGN KEY (playerId) REFERENCES Player(playerId),
  FOREIGN KEY (teamId) REFERENCES Team(teamId)
);

-- -----------------------------
-- Insert Data
-- -----------------------------
INSERT INTO Team (teamId, name, city, stadium) VALUES
(1, 'Real Madrid', 'Madrid', 'Santiago Bernabéu'),
(2, 'FC Barcelona', 'Barcelona', 'Olympic Stadium'),
(3, 'Liverpool', 'Liverpool', 'Anfield'),
(4, 'Manchester City', 'Manchester', 'Etihad Stadium'),
(5, 'Paris Saint-Germain', 'Paris', 'Parc des Princes'),
(6, 'AC Milan', 'Milan', 'San Siro');

INSERT INTO Player (playerId, fullName, teamId, position, birthYear) VALUES
(1,  'Vinícius Júnior', 1, 'FW', 2000),
(2,  'Jude Bellingham', 1, 'MF', 2003),
(3,  'Robert Lewandowski', 2, 'FW', 1988),
(4,  'Pedri', 2, 'MF', 2002),
(5,  'Mohamed Salah', 3, 'FW', 1992),
(6,  'Virgil van Dijk', 3, 'DF', 1991),
(7,  'Erling Haaland', 4, 'FW', 2000),
(8,  'Kevin De Bruyne', 4, 'MF', 1991),
(9,  'Kylian Mbappé', 5, 'FW', 1998),
(10, 'Marquinhos', 5, 'DF', 1994),
(11, 'Alisson Becker', 3, 'GK', 1992),
(12, 'Theo Hernández', 6, 'DF', 1997),
(13, 'Paulo Souza', NULL, 'FW', 1999),
(14, 'İlkay Gündoğan', 2, 'MF', 1990);

INSERT INTO `Match` (matchId, homeTeamId, awayTeamId, matchDate, stadium, homeGoals, awayGoals) VALUES
(1, 1, 2, '2025-09-01', 'Santiago Bernabéu', 2, 1),
(2, 3, 5, '2025-09-05', 'Anfield', 0, 0),
(3, 2, 4, '2025-09-10', 'Olympic Stadium', 2, 3),
(4, 1, 5, '2025-09-15', 'Santiago Bernabéu', 1, 2),
(5, 4, 3, '2025-09-20', 'Etihad Stadium', 1, 1),
(6, 5, 2, '2025-09-25', 'Parc des Princes', 2, 0),
(7, 1, 3, '2025-10-02', 'Santiago Bernabéu', 0, 1),
(8, 4, 5, '2025-10-10', 'Etihad Stadium', NULL, NULL);

INSERT INTO Appearance (matchId, playerId, teamId, minutes, goals, assists) VALUES
(1,1,1,90,1,0),(1,2,1,88,1,1),(1,3,2,90,1,0),(1,4,2,90,0,1),
(2,5,3,90,0,0),(2,6,3,90,0,0),(2,9,5,90,0,0),(2,10,5,65,0,NULL),
(3,3,2,90,1,0),(3,4,2,85,1,1),(3,7,4,90,2,0),(3,8,4,88,1,1),
(4,1,1,90,0,1),(4,2,1,90,1,0),(4,9,5,90,1,0),(4,10,5,90,1,0),
(5,7,4,90,1,0),(5,8,4,80,0,1),(5,5,3,90,1,0),(5,6,3,90,0,0),
(6,9,5,90,1,1),(6,10,5,90,1,0),(6,3,2,90,0,0),(6,4,2,78,0,NULL),
(7,1,1,90,0,0),(7,2,1,75,0,NULL),(7,5,3,90,1,0),(7,6,3,90,0,0);

-- -----------------------------
-- Queries
-- -----------------------------

-- 1) Teams ordered by city, then name
-- SELECT teamId, name, city, stadium
-- FROM Team
-- ORDER BY city, name;
-- Expected: 6 rows ordered by city alphabetically

-- 2) Teams whose stadium name contains 'Park'
-- SELECT teamId, name, stadium
-- FROM Team
-- WHERE stadium LIKE '%Park%';
-- Expected: none with current data

-- 3) Free agents (teamId IS NULL)
-- SELECT playerId, fullName, teamId
-- FROM Player
-- WHERE teamId IS NULL
-- ORDER BY fullName;
-- Expected: playerId 13 | Paulo Souza | NULL

-- 4) Matches where homeGoals = awayGoals (including NULL=NULL)
-- SELECT matchId, homeTeamId, awayTeamId, matchDate, homeGoals, awayGoals
-- FROM `Match`
-- WHERE homeGoals <=> awayGoals
-- ORDER BY matchId;
-- Expected: matches 2, 5, 8

-- 5) Players with appearances and total goals
-- SELECT p.playerId, p.fullName, COALESCE(SUM(a.goals),0) AS totalGoals
-- FROM Player p
-- JOIN Appearance a ON a.playerId = p.playerId
-- GROUP BY p.playerId, p.fullName
-- HAVING COUNT(*) >= 1
-- ORDER BY totalGoals DESC, p.fullName;
-- Expected: top = Haaland(3), then Bellingham, Lewandowski, Mbappé, Marquinhos, Salah(2 each), etc.

-- 6) Players with at least 2 appearances
-- SELECT p.playerId, p.fullName, COUNT(*) AS appearancesCount
-- FROM Player p
-- JOIN Appearance a ON a.playerId = p.playerId
-- GROUP BY p.playerId, p.fullName
-- HAVING COUNT(*) >= 2
-- ORDER BY appearancesCount DESC, p.fullName;
-- Expected: players 1–10 with 2–3 appearances

-- 7) Matches on/after 2025-09-10: (matchDate, fullName, teamName, minutes, goals)
-- SELECT m.matchDate, p.fullName, t.name AS teamName, a.minutes, a.goals
-- FROM Appearance a
-- JOIN `Match` m ON m.matchId = a.matchId
-- JOIN Player p ON p.playerId = a.playerId
-- JOIN Team t ON t.teamId = a.teamId
-- WHERE m.matchDate >= '2025-09-10'
-- ORDER BY m.matchDate, p.fullName;
-- Expected: all appearances for matches 3–7 (since >= 2025-09-10)

-- 8) Players who have never made an appearance
-- SELECT p.playerId, p.fullName, p.teamId
-- FROM Player p
-- LEFT JOIN Appearance a ON a.playerId = p.playerId
-- WHERE a.playerId IS NULL
-- ORDER BY p.playerId;
-- Expected: 11 (Alisson Becker), 12 (Theo Hernández), 13 (Paulo Souza), 14 (İlkay Gündoğan)

