The documentation you are reading now is made of two things:
- A Markdown documentation
- A [Doxygen Documention](../doxygen/classes.html)

# Markdown documentation
The [Markdown documentation](http://lis-epfl.github.io/MAVRIC_Library/index.html) is the documentation you are reading now. It contains additional information on how to use a MAV'RIC autopilot, and how to use the library to develop a custom autopilot. 

It is hand-written in the Markdown (.md) format and automaticcally converted to HTML and deployed online.

### Organisation of source files
The source files are in the [`MAVRIC_Library/doc/docs` folder](https://github.com/lis-epfl/MAVRIC_Library/tree/master/doc/docs) of the Library:
- Each folder corresponds to a subsection in the left panel of the documentation.
- Each Mardown (.md) file corresponds to a documentation page.
- The numeral prefixes ('00_', '01_', ...) on folder names allow to sort the menu.
- The numeral prefixes on file names allow to sort the pages within the current subsection.
- In each folder/subsection, you can create a so-called *landing page* to introduce the content of this folder. All you need to do is add a file named **index.md** to the folder.

### Preview the Website
An HTML version is generated each time a commit is pushed to master, then automatically deployed on the website.
In order to preview your changes before they are merged to master and automatically deployed to the website, you can preview the website on your computer. To do so: 
1) Go to the **doc/daux.io** folder and start a PHP server on your computer:
```bash
cd doc/daux.io
./serve
``` 
2) Open the generated website at [http://localhost:8085](http://localhost:8085).
This preview is live, i.e., changes in the files will directly affect the preview website once the file is saved.

3) To stop the server, press `Ctrl-C` in the console where the server is running.

### How to Contribute
To contribute to the Markdown Documentation, the method is similar to development of software:
1) From branch **master**, create a new branch named **doc/xxx_xxx** where **xxx_xxx** briefly describes your contribution to the documentation. Example **doc/code_architecture** or **doc/create_task**.
```bash
git checkout master
git pull
git checkout -b doc/xxx_xxx
```
2) Update the Markdown files with your favourite text editor
4) Commit your changes and push them on the online github repository
```bash
git add file1.md file2.md
git commit -m "Brief description of your contribution"
git push
```
5) Create a pull request ([here is](https://help.github.com/articles/creating-a-pull-request/) how to do) and ask someone to review your changes.
6) Once your pull request is merged, your contribution is online!

# Doxygen Documentation
A [Doxygen Documention](http://lis-epfl.github.io/MAVRIC_Library/doxygen/classes.html). It contains information on structures, classes and associated functions present in the Library. It also shows dependency graphs for classes and call graphs for functions. 

The content is extracted from comments in the code. The HTML pages are automatically generated each time a commit is pushed to master, then deployed online.

### Preview the Website
Doxygen handles all the formating, so it is not really required to preview the generated pages, but in case you need to do it, here is how:
1) Go to the **travis** folder to compile the doxygen documentation:
```bash
cd travis/
make doc_doxygen
cd ../
```

2) Open the generated hmtl files
```bash
firefox doc/gh_pages/doxygen/index.hml
```

### Contribute
The only thing to do is to add nice documentation in the comments of your code!
