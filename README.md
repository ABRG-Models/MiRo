# MiRo
For collaborative development of the MiRo-E demo code

## Getting started
There's just a few things you'll need to do before using this repo for MiRo development:
1. Request access to the repo in the Slack and accept the invite (if you can see this you've already done that!)
2. Clone the repo to a suitable location on your local machine
3. Re-run the MDK installer (at `mdk/bin/deb64/install_mdk.sh`) from the download location to set this version of the MDK as the one your MiRo simulator will run -- if you've previously installed the MDK you may have to remove the current symlink at `~/mdk` first
4. That's it! If you're already familiar with Git, feel free to create a branch for your project and dig in; if you've not used Git before then there's a couple of handy guides linked below, but just ask if you need any help and in general please make sure that any changes are commented and that code merged into the 'master' branch is fully functional.

* [Resources to help learn Git](https://try.github.io)
* [Guides to various Git concepts](https://guides.github.com)
* [Git commands cheatsheet](https://github.github.com/training-kit/downloads/github-git-cheat-sheet/)

### MSc students
The desktop computers in B.09 have a copy of this repo at `~/Documents/MiRo`; the first thing you should do is create and checkout a new branch for your project by changing to the repo directory and running `git checkout -b <your_project_name>`. We're going to store project work in a single cloned repo location on the disk (as this makes running the MDK a lot easier) but in separate branches, so it's probably worth familiarising yourself with [how branching works in Git](https://git-scm.com/book/en/v2/Git-Branching-Branches-in-a-Nutshell) if you've not used Git before. In particular, it's important that you remember to *checkout* your branch each time you want to do some work, and then *commit* and *push* your changes once you're finished.
