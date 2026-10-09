# stm32-car

Firmware for the Team 7 self-driving car: one STM32CubeIDE project for the
NUCLEO-F446RE board.

**`stm32-car` is the team's main branch.** The old `main` and `hakon-backup`
branches are kept for history only. Don't work on them.

```
stm32-car/
├── Core/Src/        our .c files (main.c and friends)
├── Core/Inc/        our .h files
├── Core/Startup/    startup assembly (generated, leave alone)
├── Drivers/         ST HAL + CMSIS libraries (generated, leave alone)
├── stm32-car.ioc    pins, clocks and peripherals (CubeMX config)
├── STM32F446RETX_FLASH.ld / _RAM.ld   linker scripts
└── .project, .cproject, .mxproject, .settings/   CubeIDE project files
```

`Debug/` appears when you build. Git ignores it.

---

## First-time setup

Do this once per computer. It takes about 10 minutes.

### 1. Install the tools

1. **STM32CubeIDE 1.17.0.** Everyone must use the same version: opening
   `stm32-car.ioc` in a different version converts the file and causes
   trouble for everyone else.
2. **Git.**
   - Windows: install [Git for Windows](https://git-scm.com/download/win)
     and keep all the default options.
   - macOS: open Terminal and run `git --version`. It offers to install Git
     if it's missing.
   - Linux: install `git` with your package manager.
3. Tell Git who you are. Use the email address of your GitHub account:
   ```
   git config --global user.name "Your Name"
   git config --global user.email "you@example.com"
   ```
4. Make sure you are a collaborator on
   [SondreTH/stm32-car](https://github.com/SondreTH/stm32-car). Ask Sondre if
   you can't push.

### 2. Remove any old copy of the project from CubeIDE

Skip this step if you have never opened this project before.

1. Push or back up anything in your old copy you want to keep.
2. In CubeIDE's **Project Explorer**, right-click the old project. It may be
   called `Hakons_leg`, `Car7_A01832890`, `Team7_Car`, `selfdrivingcar` or
   similar.
3. Click **Delete**. **Leave "Delete project contents on disk" unticked**, then
   click **OK**. This only removes the project from CubeIDE's list; the files
   stay on your disk.

### 3. Clone the repository

1. Open a terminal in the folder where you want to keep the code.
   - Windows: open the folder in File Explorer, right-click an empty spot, and
     choose **Open Git Bash here**. On Windows 11, choose **Show more options**
     first.
   - Windows: don't use a folder synced by OneDrive (by default this often
     includes Documents and Desktop). OneDrive locks files while the code is
     building. `C:\dev` works well.
2. Run:
   ```
   git clone -b stm32-car https://github.com/SondreTH/stm32-car.git
   ```
   This creates a folder called `stm32-car`. **Don't rename it.** CubeIDE
   needs the folder name to match the project name.

Never open the `stm32-car` folder itself as a CubeIDE workspace (through
**File → Switch Workspace**). Your workspace is the separate folder CubeIDE
asks for when it starts.

### 4. Import the project into CubeIDE

1. Start STM32CubeIDE with your normal workspace.
2. Go to **File → Import…**.
3. Choose **General → Existing Projects into Workspace**, then click **Next**.
4. Next to **Select root directory**, click **Browse…** and choose the
   `stm32-car` folder you just cloned.
5. Under **Projects**, `stm32-car` should be ticked. Under **Options**, make
   sure **Copy projects into workspace** is **unticked**.
6. Click **Finish**. `stm32-car` now shows in Project Explorer.

### 5. Build

1. Click `stm32-car` in Project Explorer.
2. Go to **Project → Build Project**.
3. The Console should end with `Build Finished. 0 errors, 0 warnings.`

### 6. Flash the board

1. Plug the Nucleo board into your computer over USB.
2. Right-click `stm32-car` and choose **Run As → STM32 C/C++ Application**.
3. The first time, a configuration window opens. The defaults are correct, so
   just click **OK**.
4. The Console shows `Download verified successfully` and the board starts
   running the new code.

After the first time, the green **Run** button (▶) builds and flashes in one
click.

### If something goes wrong

| Problem | Fix |
|---|---|
| Import says the project already exists in the workspace | Do step 2 first, then import again. |
| `ST-LINK firmware upgrade required` | Go to **Help → ST-LINK Upgrade**, click **Refresh device list → Open in update mode → Upgrade**, then unplug and replug the USB cable. |
| Linux: `libncurses.so.5: cannot open shared object file` | Install the ncurses 5 compatibility package. On Arch it's `ncurses5-compat-libs`. |
| `No ST-LINK detected` | Try another USB cable. Many cables only charge and can't carry data. |

---

## Everyday workflow

Before you start working:

```
git pull
```

Then in CubeIDE, right-click `stm32-car` and choose **Refresh** (F5), so the
IDE sees the new files.

When something works:

```
git add -A
git commit -m "Short description of what you changed"
git pull --rebase
git push
```

### Rules

1. **In generated files, only write code between `USER CODE BEGIN` and
   `USER CODE END`.** The generated files are `main.c`, `main.h`,
   `stm32f4xx_it.c` and `stm32f4xx_hal_msp.c`. Anything outside those
   markers is erased the next time someone generates code from the `.ioc`.
2. **Put new code in new files.** Add `.c` files to `Core/Src` and `.h`
   files to `Core/Inc`. CubeIDE compiles them automatically, with no project
   settings to change.
3. **Only one person edits `stm32-car.ioc` at a time.** Tell the team before
   you change pins or peripherals, and push right after you click
   **Generate Code**. Two people regenerating at the same time causes
   conflicts in many generated files.
4. **Pull before you start, and push small working changes often.**
5. **Never use `git push --force`.**

### If `git pull --rebase` reports a conflict

1. Open each file Git lists and find the `<<<<<<<` / `=======` / `>>>>>>>`
   blocks.
2. Edit each block so the file contains the code you want, and delete the
   marker lines.
3. Run `git add <file>` for each file you fixed, then `git rebase --continue`.

Not sure what to keep? Run `git rebase --abort` to get back to where you
were before pulling, and ask in the group chat.
