# learn_ros2_code
learn ros2

youtube link: https://www.youtube.com/watch?v=Gg25GfA456o&list=PLLSegLrePWgJk6dfV-UXSh2TZ74wNntWt&ab_channel=RoboticsBack-End  

ros2 doc: https://docs.ros.org/en/humble/index.html  

in china, network is bad, install ros2 may have problem, use proxy  

for me, i have problem when install ros2 follow the doc https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Install-Binary.html

in add the ROS 2 GPG key with apt step   
  sudo apt update && sudo apt install curl -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg  
i can not get https://raw.githubusercontent.com/ros/rosdistro/master/ros.key in shell,   
to slove this problem, copy the link to brower, download ros.key file , copy the file to /usr/share/keyrings/ros-archive-keyring.gpg
