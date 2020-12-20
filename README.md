# DroneIVSR

Project has create via guide on https://gaas.gitbook.io/.

In this project has full file after installed and setup

![46384ca9aa0c56520f1d](https://user-images.githubusercontent.com/69444682/90326110-8e203380-dfae-11ea-8793-686585254def.jpg)

![image](https://user-images.githubusercontent.com/69444682/102704197-bdbbb100-42aa-11eb-84f6-1e8e7b59f865.png)

### Remove Folder on your local repository.
The steps for doing this are:

* In the command-line, navigate to your local repository.
* Ensure you are in the default branch:
```shell
 $ git checkout master
```
* The rm -r command will recursively remove your folder:
```
 $ git rm -r folder-name
```
* Commit the change:
```
 $ git commit -m "Remove duplicated directory"
```
* Push the change to your remote repository:
```
 $ git push origin master
```

### Update your local branch "master"
```shell
 $ git pull origin master
```
