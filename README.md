# 2D Lidar odometry in the Real-World 
- [2D Lidar odometry in the Real-World](#2d-lidar-odometry-in-the-real-world)
  - [Project description](#project-description)
  - [Implementation details](#implementation-details-and-result)
  - [Installer Details](#installation)
    - [Ubuntu](#ubuntu)
  - [Acknowledgements](#acknowledgements)

  ## Project Description

  In this project, Iterative Closest Point(ICP) algorithm has been implemented to register consecutive  
  scans taken from a 2D Laser Scanner mounted on a vehicle.  


  ## Implementation Details and Result
  Functions implemented for registering laser scans are generic. However, there are some hypermeters such as grid-size, downsample_gridsize,and repetition score which are data specific and can be modified as per the data specifications. I have set them in main app code for my own data. Final result looks like this in my case:  

  ![Registered scans](/results/final_map.png)  
  
  Moreover, the requirement was to register the scans at 10HZ which has been achieved.  
      

  ## Istaller Details
  ### Ubuntu 
   First, you need to clone this project in any folder. You need two external libraries, Eigen3d and Open3d, to compile and execute the code successfully. Eigen3d can be installed using the following command:  
   - `sudo apt install libeigen3-dev` 

  Open3d has to be downloaded from github and extracted to the project folder. Open3d can be downloaded from [Open3d github](https://github.com/isl-org/Open3D/releases). For ubuntu system, download this one _open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz_ from the releases. Extract it to project directory and rename the folder as "open3d".  
  For Data, you have to create a folder "BINARY" and put all laserscaner files in it. Without BINARY folder the tree of the project is like this:  

  ![Registered scans](/results/project_tree.png)  

  Finally to build and excecute the project with CMake, following steps are required:  
    - make a directory(build) in the project folder(mkdir build)
    - go to the build directory(cd build)
    - write configuration files to build (cmake ..)
    - compile the code(make)
    - run the program(./app/main)  
  ## Acknowledgements
  Thanks to [contributors](https://github.com/SajjadHussain-UniBonn/2D-Lidar-odometry-in-the-Real-World/graphs/contributors) for making it happen.  