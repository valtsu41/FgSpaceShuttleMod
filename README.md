# FlightGear Space Shuttle Utility mod
This is a mod for the FlightGear Space Shuttle.
Original code can be found at https://sourceforge.net/projects/fgspaceshuttledev/ (on the development branch).
This repository has a separate Git history due to some very large files present in the main history.

List of current modifications:
- Separate display update rate setting for displays used as PFDs
- Improved some checklist conditions
- Following canvas dialogs now work even when paused
    - Propellant
    - Shuttle view manager
    - Trajectory map
- Radiators will not overcool freon
- Mission control HAC callouts
- Experimental new landing sites
- No automatic gear extension
- Fixed INRTL ADI yaw
- Experimental improved nominal MECO or contigency abort detection
- Dialog for editing component conditions (failures)

## Installation
Because some files in this repository are too large to be normal GitHub tracked files, this repository uses Git LFS. You need to have it set up correctly for git cloning to work.
Downloading the repository as a zip file and then unzipping it in your aircraft directory should work as usual. Make sure to rename the directory to `SpaceShuttle`.