We are using a version control tool called GIT. It is a very nice tool to collaborate in an easy and efficient way. The Code is currently on github.com/lis-epfl/ server.
If you are not yet familiar with GIT, we advise to have a quick look to the [online book](http://git-scm.com/book/en/Getting-Started-About-Version-Control) explaining the basics

### To clone the Firmware on your laptop
You will only do it once
* Open Git-bash
* Go in the folder you would like to put the mavric firmware in (eg. `cd Documents/EPFL/`)
* Type `git clone --recursive git@github.com:lis-epfl/MAVRIC.git`, followed by Enter
* Type your github password

It will clone the Git MAVRICrepository on your laptop. So you get a local copy of the MAV'RIC working environment.

Since there are some folder that are links to other repositories such as : MAVRIC_LIBRARY,mavlink,..
* in Git-bash
* go in MAVRIC git repository
* type : `git submodule init`
* and `git submodule update`
* Do the same (git submodule init & git submodule update) in mavric\MAVRIC_Library
It will update your submodules (eg. MAVRIC/MAVRIC_LIBRARY) folder to current status of those repo.

If git submodule update failed, try that:
https://help.github.com/articles/error-permission-denied-publickey/

Your own repository will be a branch of MAVRIC, let's call this branch 'mycode'
* in Git-bash
* go in MAVRIC git repository
* type: 'git checkout -b mycode'

It will create a branch called mycode and you will be in that branch as a working branch.
Please read [branch info](https://git-scm.com/book/fr/v1/Les-branches-avec-Git-Brancher-et-fusionner%C2%A0%3A-les-bases) for more information on branch management.

Then, go in MAVRIC/LEQuad, For windows users right click on mklink_Library.bat and run it as administrator. 
For Linux users type `ln -s ../MAVRIC_Library Library`.
It will create a symbolic link to MAVRIC_LIBRARY in the LEQuad folder.  

### To get the Latest firmware update
You will do it each time you want to benefit of released contributions from other developers.
* Open Git bash
* Go in your MAVRIC folder
* Type `git pull`
* Type your github password
* Check the output, to see what you get and whether any conflicts occurred.

If and only if, some conflicts happen:
* Check which file(s) is concern
* Type `git mergetool`, followed by Enter.
It will use your default merging tool (eg tortoisemerge)
The conflict are highlighted in red
* Select red blocks (conflict part of the code)
It will show what is their revision and yours
* Decide what to do:

1. use their block
1. use your block 
1. use one block and edit it to use part of the other block

* Do this for all conflict files
* Close your mergetool
* Go in Git bash, and type `git commit` to finish the merge

### Your are done