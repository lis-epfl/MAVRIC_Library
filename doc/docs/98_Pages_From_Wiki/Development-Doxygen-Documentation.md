# Doxygen documentation

The doxygen-generated documentation is online at [[http://lis-epfl.github.io/maveric/]]

This website lives in the branch 'gh-pages' of the repository and is hosted by github. To make it easier to update the documentation, a submodule 'html' is present in the folder Documentation, it points to the branch gh-pages.

## To update the documentation:

- Go to repo folder:
```
cd ~/your_path_to_mavric/MAVRIC/
```

- Reattach submodule to the branch gh-pages:
```
cd Documentation/Doxygen/html
git checkout gh-pages
```

- Generate documentation:
```
cd ../
doxygen
```

- Commit changes in documentation
```
cd html/
git commit -am "Update doc"
git push origin gh-pages
```

- Commit changes to submodule in main repo
```
cd ../
git add html
git commit -m "Update doc submodule"
git push origin master
```
