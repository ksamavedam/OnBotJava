# OnBotJava
OnBot Java Repository
## Download and Install
- Download and install: https://desktop.github.com/  
  - Create GitHub account and let me know your id; I need to add your id to it as a collaborator
- Download and install: https://code.visualstudio.com/download
  - WINDOWS:
    - Create a folder FTC in your Documents 
    - Download the ftc_http_win.exe into this folder
  - MAC:
    - TBD

- Download corresponding binaries to your computer: https://github.com/TheLostLambda/ftc_http/releases/tag/v1.3.0
-- We will use the tools to upload and download from RC : https://github.com/TheLostLambda/ftc_http

## Setup Instructions
- You will all be working on a branch, not *main* 
- You can create a branch and request for merge to *main* for team's consumption. 
- Clone this repository using GitHub Desktop
  - Create folder Documents > FTC > SkyStoneCode 
  - File->Clone Repository
  - Select Documents > FTC > SkyStoneCode as *Local Path*
  - C&P this link for cloning: https://github.com/ksamavedam/OnBotJava.git
  - ![GitHub Clone](https://github.com/ksamavedam/OnBotJava/blob/master/Misc/GitHubClone-Annotation%202019-10-20%20101012.jpg)
- Open VSCode and  drag and drop *SkyStoneCode* folder into VS Code
  - Then you will see the folder hierarchy 
  - ![Double click on any file, it will open in the editor] (https://github.com/ksamavedam/OnBotJava/blob/master/Misc/VSCode-Folder-Annotation%202019-10-20%20101012.jpg)
- ![In VSCode -> Open Terminal (see top menu) in which you will execute ftp_http commands](https://github.com/ksamavedam/OnBotJava/blob/master/Misc/VSCode-Annotation%202019-10-20%20101012.jpg)


## *When using this method to for software development, close all the browsers that are on the OnBotJava sessions*

## Coding and Testing
- Assumption is that GitHub has latest of all the code bases on different branches. 
- Get the latest code from GitHub and switch to your branch.
- If needed, merge *main* to your branch (GitHub Desktop: Branch->Merge into Current Branch, Click on *master* and merge)
- Now VS Code will have your latest code from your branch.
- Now Connect to RC Wi-Fi   
- First cleanup phone ..\ftc_http_win.exe -e  
- Working with ftc_http application
  - Modify the code (that is in your VS Code)
  - Upload your code ..\ftc_http_win.exe -u
  - Compile your Code ..\ftc_http_win.exe -b
  - Upload and Compile in one step: ftc_http_win.exe -ub
  - ![Here is a session of these commands in VSCode](https://github.com/ksamavedam/OnBotJava/blob/master/Misc/VSCode-Term-ftc-http-Annotation%202019-10-20%20101012.jpg)
  - ![Here is a session of ERROR Scenario VSCode](https://github.com/ksamavedam/OnBotJava/blob/master/Misc/VSCode-Term-ftc-http-error-Annotation%202019-10-20%20101012.jpg)

- ![You can do periodic commits from the VSCode itself] (https://github.com/ksamavedam/OnBotJava/blob/master/Misc/VSCode-GIT-Annotation%202019-10-20%20101012.jpg)
- ![At the end of the day, you must push changes to GitHub](https://github.com/ksamavedam/OnBotJava/blob/master/Misc/VSCode-GIT-Push-Annotation%202019-10-20%20101012.jpg)
- Make a *Pull Request* if you think your code can be merged to *main*
  

