# HelicalGear

This is a "Branch" of Ross Korsky's Helical Gear Generator Add-in for Autodesk's Fusion 360.
His version can be downloaded [here](https://apps.autodesk.com/FUSION/en/Detail/Index?id=9029586664984391977&os=Mac&appLang=en).
<br>
<br>

# Changes

## Timeline Grouping
This Add-In may generate hundreds of features in order to create some gears.
These features are now contained in a single timeline group so they do not clutter up the timeline.

## "Better" Input Persistence
Previously input persistence was achieved by saving the input values to attributes. This has the disadvantage of being rolled back by undos. So if you wanted to tweak a gear you just generated you would have to re-enter everything.
The new system just stores values in a variable. Persistence is lost if Fusion360 or the Add-In is restarted but persistence is not affected by undos. While a hybrid system would be possible, I do not think it is worth implementing.
<br>
<br>

# TO-DO
* [x] Setup version controll
* [x] Group features in Timeline to take up less space
* [x] Change persistent inputs so they won't reset after undo
* [x] Fix Errors when not capturing design History
* [ ] Add "Generate as Base feature" opting for better performance
<br>
<br>

# Installation
* Download the Project as ZIP and extract it somewhere you can find again, but won't bother you. (or use git to clone it there)
* Open Fusion360 and press ADD-INS > Scripts and Add-ins
* Select the tab Add-Ins and click the green plus symbol next to "My Add-Ins"
* Navigate to the extracted Project forlder and hit open
* The Add-in should now appear in the "My Add-Ins" list. Select it in the list. If desired check the "Run ond Startup" checkbox and hit run.
* The Command will appear as CREATE > Helical Gear
