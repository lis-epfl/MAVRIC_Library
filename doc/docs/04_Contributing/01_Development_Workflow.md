# Overview
The recommended workflow is:
* Create new branch from current master branch
* Work in your branch
* Push your branch and create a pull request
* Assign someone to review your code

The following sections give more details on each step.


# Branching 
### Working with dev branches
It is recommended to create a new branch from master each time you want to make changes to the code (new feature, bug fixing, code refactoring, ...).

The following commands creates a branch called **bugfix/annoying_error**
```bash
git checkout master
git pull
git checkout -b bugfix/annoying_error
```

### Naming conventions for branches
Branch names should be composed of a prefix and descriptive name separated by a forward slash.The descriptve name should be short and use underscore to separate words:

**prefix/descriptive_name**

List of prefixes:
- **bugfix/** for commits fixing bugs or issues
- **feature/** for new features
- **release/** is used to keep track of stable releases (ex:**release/v1.5.x**)
- **doc/** for documentation

# Creating a pull request
When you are happy with the changes you made on your computer on your new branch, you can share your work with the others. To do that: 
1) Check which files where changed with
```bash
git status
```

2) Commit you changes (here file1.ccc and file1.hpp were modified):
```bash
git add file1.cpp file1.hpp
git commit -m "Awesome feature"
```
3) Push to the online github repository
```bash
git push 
```
4) Create a pull request (see [here](https://help.github.com/articles/creating-a-pull-request) for more information)
5) Assign someone to review your changes, possibly make comments and ask you to correct things, and finally merge your changes to the branch master.