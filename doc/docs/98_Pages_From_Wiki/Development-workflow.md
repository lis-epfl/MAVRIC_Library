# Development workflow
######################

## I) Overview
In shot the recommended workflow is:
* Pull new changes
* Create new branch
* Work in your branch
* Push your branch
* Create a pull request

The following sections give more details on each step.


## II) Branching 
### II-1) Working with dev branches
It is recommended to create a new branch from master each time you want to make changes to the code (new feature, bug fixing, code refactoring, ...).

- Create a branch from master (the following command creates a branch called *bugfix/my-new-branch*)
```
git checkout master
git branch bugfix/my-new-branch
git checkout bugfix/my-new-branch
```
or the shorter version
```
git branch -d bugfix/my-new-branch master
```

### II-2) Naming conventions for dev branches
Branches should be named with a prefix and a descriptive name with underscores, separated by a forward slash (```prefix/descriptive_name```).
List of prefixes;
- ```bugfix``` : 
- ```feature``` : 
- ```junk``` : 

## III) Working on the Library
### III-1) I have push access to ```MAVRIC_Library```
**Do not push to master!**


### III-2) I do not have push access to ```MAVRIC_Library```

For users that do not have push access to the repository ```MAVRIC_Library```, we created a repository called ```MAVRIC_Library_dev```. This is a development version of the Library. Changes mades in this repository should be merged upstream with a pull-request.

Recommended workflow:
- Clone this dev repository
```
git clone git@github.com:lis-epfl/MAVRIC_Library_dev.git
```
- Create a new branch according to your username and the name of the feature you are developping 
```
cd MAVRIC_Library_dev
git branch -d feature/username_NewFeature
git push --set-upstream origin feature/username_NewFeature
```
- Work on your computer
- Commit changes
```
git add file1.c file2.c file3.c
git commit -m "Awesome feature"
```
- Push to the dev repository
```
git push 
```
- Create a pull request (see https://help.github.com/articles/creating-a-pull-request)
- Wait for someone with push access to MAVRIC_Library to:
    * review your changes
    * possibly make comments and ask you to correct things
    * merge your changes to MAVRIC_Library