# Add-In installation troubleshooting guide

Please find the section relating to your Issue and try these steps. If your issue is not listed here or the steps listed here do not fix it then please send me the following information:

- Add-In name & version
- Operating system
- At what stage of the installation the Issue occurs
- Any error messages produced

## 1. The installer does not launch

### a. (macOS only)
 - Open the installer with Right Click > Open instead of double-clicking it


## 2. The installer fails or completes but does not actually install the Add-In

### a. Clean reinstall

If the installer fails, or a previous version was not deleted correctly files can be left behind preventing the installer from working.

 1. If the Add-In appears in the “Scripts and Add-Ins” menu inside Fusion
  - Select the Add-In in the “Scripts and Add-Ins” menu inside Fusion
  - Right Click > Open file location
  - Delete the entire folder (or .bundle file on macOS) that opened. (It should have the same name as the Add-In)
  - Select the Add-In in the “Scripts and Add-Ins” menu inside Fusion
  - Right Click > Remove
2. If the Add-In does not appear in the “Scripts and Add-Ins” menu inside Fusion
 - (macOS only)
     - Navigate to Add-In installation folder (~/Library/Containers/com.autodesk.mas.fusion360/Data/Library/Application Support/Autodesk/ApplicationPlugins)
     - Delete the .bundle file with the name of the Add-In, if it exists
 - (Windows only)
     - Navigate to Add-In installation folder 
     - Delete the folder with the name of the Add-In, if it exists 
3. Re-run the installer

### b. Manual install

If the installer still does not want to cooperate a manual install is rather easy.

1. Install “Install scripts or addins from GitHub” Tool by Gerome Briot from the Fusion360 App store

2. It should appear as “GitHubToFusion360” in the “Scripts” section of the "Scripts and Add-Ins” menu.
Run it, and when asked for an URL enter:
 - (Helical Gear Plus only)
     - https://github.com/NicoSchlueter/HelicalGearPlus
3. Once completed the Add-In should appear in the "Scripts and Add-Ins” menu.
Check the “Run on startup” checkbox and press the run button.
