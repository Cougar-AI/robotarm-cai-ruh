# Linux Basics — Workshop Handout



---

## Before you start

You need a real Linux shell. Pick whichever is easiest:

- **Mac** → open Terminal (it's Unix, close enough for this workshop)
- **Windows** → install WSL2 (`wsl --install` in PowerShell, then restart)
- **Linux** → you're already there
- **Chromebook / locked-down machine** → use [replit.com](https://replit.com) or [killercoda.com](https://killercoda.com) in your browser

---

## 1. What a shell actually is

When you open a terminal, two things are happening:

- **The terminal** is the window — just a box that shows text.
- **The shell** is a program running inside it that reads what you type, runs it, and prints the result.

Most Linux systems use `bash` or `zsh` as the default shell. You can check yours:

```bash
echo $SHELL
```

Every command follows the same shape:

```
$ ls -la /etc
  │  │   │
  │  │   └── arguments — what you want to act on
  │  └────── flags (options) — usually start with - or --
  └────────── command — the program to run
```

The `$` is the prompt. You don't type it — it's just the shell saying "I'm ready."

---

## 2. The filesystem

Linux doesn't have `C:\` or `D:\`. There is **one tree**, starting at `/` (the root).

```
/
├── home/
│   └── tom/        ← your stuff lives here
├── etc/            ← system-wide configuration
├── usr/            ← installed programs and libraries
├── var/            ← logs, mail, caches
└── tmp/            ← temporary files (wiped on reboot)
```

Things to remember:

- Paths use forward slashes: `/home/tom/projects`
- `~` is shorthand for your home directory
- `.` means "the current directory"
- `..` means "the parent directory"
- Everything is **case-sensitive** — `File.txt` and `file.txt` are different files

---

## 3. Navigation

| Command | What it does |
|---|---|
| `pwd` | Print working directory (where am I?) |
| `ls` | List contents of current directory |
| `ls -l` | Long format — sizes, dates, permissions |
| `ls -la` | Same, but include hidden files (those starting with `.`) |
| `cd foo/` | Change into directory `foo` |
| `cd ..` | Go up one level |
| `cd ~` | Jump to your home directory |
| `cd -` | Return to the previous directory you were in |

**Walkthrough:**

```bash
$ pwd
/home/tom

$ ls
Desktop  Documents  projects

$ cd projects
$ pwd
/home/tom/projects

$ cd ..
$ pwd
/home/tom
```

---

## 4. Creating, moving, and deleting files

| Command | What it does |
|---|---|
| `mkdir notes` | Create a directory called `notes` |
| `mkdir -p a/b/c` | Create nested directories in one shot |
| `touch file.txt` | Create an empty file (or update its timestamp) |
| `cp src.txt dst.txt` | Copy a file |
| `cp -r src/ dst/` | Copy a whole directory recursively |
| `mv old.txt new.txt` | Move *or* rename (same command) |
| `rm file.txt` | Delete a file |
| `rm -r folder/` | Delete a directory and everything inside |

> ⚠️ **There is no trash.** `rm` is permanent. Double-check what's in the folder before you delete it. `rm -rf /` is the classic disaster — don't run anything like that casually.

---

## 5. Reading file contents

| Command | When to use it |
|---|---|
| `cat file.txt` | Dump the whole file to the screen — good for short files |
| `less file.txt` | Paged viewer — arrow keys to scroll, `q` to quit |
| `head file.txt` | First 10 lines (`head -n 20` for 20 lines) |
| `tail file.txt` | Last 10 lines |
| `tail -f app.log` | **Follow** a file as it's written — essential for watching logs |

```bash
$ cat /etc/os-release      # see what distro you're on
$ tail -f /var/log/syslog  # watch system messages live (Ctrl+C to stop)
```

---

## 6. Permissions

Run `ls -l` and you'll see lines like this:

```
-rwxr-xr--  1 tom  staff  2048  Apr 23  script.sh
 │││ │││ │││
 │││ │││ └┴┴── others can: read only
 │││ └┴┴────── group can: read + execute
 └┴┴────────── owner can: read + write + execute
```

Three permission groups (owner / group / others), each with three flags:

- `r` — read
- `w` — write (modify)
- `x` — execute (run as a program)

A `-` means the permission is denied.

**Changing permissions with `chmod`:**

```bash
# Symbolic form — add execute permission for everyone
chmod +x script.sh

# Numeric form — each digit is a sum: r=4, w=2, x=1
chmod 744 file.txt   # owner: rwx (7), group: r (4), others: r (4)
chmod 755 script.sh  # standard for executables
chmod 644 notes.txt  # standard for regular files
```

**`sudo`**: run a command as the superuser. Required for changing system files, installing packages, etc.

```bash
sudo apt install git   # system action — needs sudo
git status             # normal action — doesn't
```

---

## 7. Pipes and redirection — the Linux superpower

Linux tools are small and single-purpose by design. You combine them with three operators:

| Symbol | Name | What it does |
|---|---|---|
| `\|` | pipe | Send the output of one command into the next |
| `>` | redirect | Write output to a file (overwrites it!) |
| `>>` | append | Write output to a file (adds to the end) |

**Examples:**

```bash
# count how many .py files are here
ls *.py | wc -l

# save all running processes to a file
ps aux > processes.txt

# append a line to a log without overwriting
echo "deploy finished" >> deploy.log

# find errors in a log, live
tail -f app.log | grep ERROR

# chain three tools: list files, find ones containing "test", count them
ls *.txt | grep test | wc -l
```

Once you internalize pipes, you start thinking of the shell as a Lego set.

---

## 8. Finding things

Two tools, often confused:

### `grep` — search **inside** files

```bash
grep "TODO" file.py          # find TODO in one file
grep -r "TODO" .             # recursive — search all files under current dir
grep -i "error" app.log      # case-insensitive
grep -n "error" app.log      # show line numbers
grep -rn "TODO" .            # combined: recursive + line numbers
```

### `find` — search **for** files by name, date, size

```bash
find ~/code -name "*.js"     # all .js files under ~/code
find . -type d -name "node_modules"   # directories named node_modules
find . -mtime -1             # files modified in the last day
find . -size +100M           # files larger than 100 MB
```

### Bonus: searching your own command history

```bash
history          # list everything you've typed
```

Press **Ctrl + R** and start typing — the shell searches your history interactively. This is the single biggest productivity upgrade once you know it exists.

---

## 9. Packages and processes

### Installing software (Ubuntu / Debian / WSL)

```bash
sudo apt update                 # refresh the package list
sudo apt install git            # install a package
sudo apt remove git             # uninstall
apt list --installed            # what's installed
apt search python               # search for packages
```

Other distros use different tools: Fedora → `dnf`, Arch → `pacman`, macOS → `brew`.

### Seeing what's running

```bash
ps aux               # snapshot of all running processes
top                  # live, interactive view (q to quit)
htop                 # nicer top, if installed
kill 1234            # politely ask process 1234 to stop
kill -9 1234         # force-kill it (last resort)
```

Every running program has a **PID** (process ID). `ps` and `top` show them.

---

## 10. Hands-on lab (15 min)

Open your terminal and work through these. Don't copy-paste — type them. Muscle memory is the whole point.

1. Create a folder called `linux-lab` in your home directory.
2. Inside it, create three files: `notes.txt`, `todo.txt`, `readme.md`.
3. Write `hello linux` into `notes.txt` using `echo` and `>`.
4. Count how many files are in the folder using `ls` and `wc`.
5. Use `grep` to find which file contains the word `hello`.
6. Rename `readme.md` to `README.md`, then delete `todo.txt`.

### Solutions (try it yourself first!)

<details>
<summary>Click to expand</summary>

```bash
# 1
mkdir ~/linux-lab
cd ~/linux-lab

# 2
touch notes.txt todo.txt readme.md

# 3
echo "hello linux" > notes.txt

# 4
ls | wc -l

# 5
grep -l hello *

# 6
mv readme.md README.md
rm todo.txt
```

</details>

**Stuck on a command?** Two universal escape hatches:

```bash
man ls        # full manual page (q to quit)
ls --help     # quick summary, works on most commands
```

---

## 11. Cheat sheet

```
NAVIGATION          FILES               VIEWING
pwd                 mkdir dir/          cat file
ls   ls -la         touch file          less file
cd dir/             cp src dst          head file
cd ..               mv src dst          tail -f file
cd ~   cd -         rm file
                    rm -r dir/

PERMISSIONS         SEARCHING           PIPES
chmod +x file       grep pattern file   cmd1 | cmd2
chmod 755 file      grep -r pattern .   cmd > file
sudo cmd            find path -name     cmd >> file
                    history / Ctrl+R

PACKAGES            PROCESSES           HELP
sudo apt update     ps aux              man cmd
sudo apt install    top / htop          cmd --help
                    kill PID
```

---

## 12. Where to go next

- **The Linux Command Line** — free book by William Shotts. The best single resource. [linuxcommand.org](https://linuxcommand.org)
- **explainshell.com** — paste any shell command, get a breakdown of what each piece does
- **tldr pages** — quick, example-first alternative to `man`. Install with `sudo apt install tldr`, then try `tldr tar`
- **OverTheWire: Bandit** — gamified shell challenges that go from total beginner to surprisingly deep. [overthewire.org/wargames/bandit](https://overthewire.org/wargames/bandit/)
- **A cheap VPS** (DigitalOcean, Hetzner, a free AWS tier) — nothing builds fluency like administering your own server

---

*The best way to learn Linux is to break things — safely. Keep a VM or WSL instance around that you're willing to nuke and reinstall. Then nuke it.*
