all.bash can be used to run multiple tests at once using a folder of configurations

Usage :
all.bash FOLDER_OF_CONFIGURATIONS BASE_YAML WORKSPACE EXECUTABLE PROCESS_COUNT

./all.bash ../dev/tests/sources ../dev/tests/base.yaml ../dev/work ../../sfs-framework.git-debug/bin/sfs-framework 4


FOLDER_OF_CONFIGURATIONS is a folder containing multiple *.yaml file. For instance 

```
frontend:
  type: IMAGE
  source:
    type: FOLDER
    path: /home/dolu/pro/scanvan/dataset/d2
    mask: /home/dolu/pro/scanvan/dataset/mask.png
    scale: 0.5
```

Please use absolute path.

BASE_YAML is the path to a file used to complet the FOLDER_OF_CONFIGURATIONS files. For instance : 

```
algorithm:
  error: 1e-6
  structure: 3
  disparity: 5
  radius: 10
export:
  path: dev/log
debuga:
  lastViewPointGui:
    structureSizeMin: 5
```

WORKSPACE is the folder where all the tests will run and where the logs will be saved
EXECUTABLE is the sfs-framework binary path
PROCESS_COUNT is how many EXECUTABLE will run at the same time
