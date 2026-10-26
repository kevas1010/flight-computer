# Running on bare-metal Linux

The numbers in the README were measured on WSL2, which is a Linux kernel inside a
Hyper-V virtual machine. Everything functional (the flight software, protocol, netem, tests) behaves
the same there, but loop timing is limited by the VM's timer. On bare metal the
same campaign measures what the hardware and the Linux scheduler really do.

Any of these works. The bootstrap script handles each one the same way.

| Machine | OS to install | Notes |
|---|---|---|
| Spare x86-64 laptop/PC | Ubuntu 24.04 LTS Desktop | Easiest. If you don't need Windows on it, choose "Erase disk and install Ubuntu". |
| Raspberry Pi 4 or 5 | Ubuntu Server 24.04 LTS (64-bit) | Runs headless over SSH. Raspberry Pi OS (64-bit) also works. |

An ESP32 or other microcontroller won't work: it can't run Linux (there's no MMU and
only ~0.5 MB of RAM).

## 1. Install the OS

**Laptop:** download Ubuntu 24.04 LTS Desktop from ubuntu.com, write it to an
8 GB+ USB stick with balenaEtcher or Rufus, boot from the stick (usually F9/F12/Esc
at power-on), and install. Dual-boot doesn't slow Windows down; it only uses disk
space (give Ubuntu ~50 GB). Before dual-booting, back up, save your BitLocker
recovery key if the drive is encrypted, and turn off Windows Fast Startup.

**Raspberry Pi 4:** you need a microSD card (16 GB+), a USB-C power supply (5 V 3 A)
and a network connection (Ethernet is simplest).
1. On your PC, install Raspberry Pi Imager from raspberrypi.com/software.
2. Choose *Raspberry Pi 4* → *Other general-purpose OS* → *Ubuntu* → *Ubuntu Server 24.04 LTS (64-bit)*.
3. Click the settings (gear / "Edit settings") and set a username and password,
   enable SSH, and enter your Wi-Fi if you won't use Ethernet.
4. Write the card, put it in the Pi, power on, wait ~2 minutes, then connect from your PC:
   `ssh <username>@<pi-hostname>.local` (or find the Pi's IP in your router's device list).

A 1–2 GB Pi works; the script automatically builds with fewer parallel jobs so it
doesn't run out of memory (the build just takes longer, ~15–25 min on a Pi 4).

## 2. Run everything

```bash
sudo apt-get update && sudo apt-get install -y git
git clone https://github.com/kevas1010/flight-computer.git
cd flight-computer
bash scripts/bootstrap.sh
```

The script installs the dependencies, builds release and ASan builds, runs all
the tests, then runs the full measurement campaign (about 15 minutes; leave the
machine idle during it, since other work skews the timing numbers). It asks for your
password once. Results go to a new folder,
`results/<date>-<host>-baremetal/`, with the summary in `summary.md` and the machine
details in `machine.txt`. At the end it prints a comparison table covering every
result set, including the WSL2 ones already in the repo.

`bash scripts/bootstrap.sh --no-measure` stops after the tests.

## 3. Add the results to your GitHub history

On the Pi or laptop, from the `flight-computer` folder:

```bash
python3 scripts/compare.py --update-readme
git add results README.md
git commit -m "Add bare-metal measurements"
git push
```

The earlier work stays where it is:

- Each campaign writes its own dated folder, and `measure.sh` refuses to write into
  an existing one, so the WSL2 sets in `results/` are never touched.
- `compare.py --update-readme` regenerates the README comparison table from **all**
  sets in `results/`, so the bare-metal rows appear next to the WSL2 rows
  instead of replacing them.
- Only small files (summaries, JSON, machine info) are committed; flight logs and
  CSVs stay on the machine (`.gitignore`).

To push from the Pi, sign in once: GitHub no longer accepts passwords for git. Use a
personal access token (GitHub → Settings → Developer settings → Tokens) or
`gh auth login` after `sudo apt-get install -y gh`. Alternatively, copy the new
`results/` folder to your PC (`scp -r <user>@<host>:~/flight-computer/results/<new-folder> results/`)
and commit it there.

If the bare-metal jitter is what you want on your resume, quote it with its
platform ("p99 X µs on a Raspberry Pi 4, bare-metal Linux"), and keep the WSL2
numbers in the README as the comparison.
