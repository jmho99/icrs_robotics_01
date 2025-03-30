# 로보틱스 / 로봇공학 (김영식 교수님) 팀프로젝트 01

## 사용 방법

노션 페이지 : <https://playful-butterkase-c7d.notion.site/1ae578c15d8b802cab64cdfc29db0ad7>

## UR ROBOT Driver / Description 불러오기

    vcs import src --skip-existing --input src/icrs_pick_and_place/icrs_pick_and_place.${ROS_DISTRO}.repos

### UR robot 정보 불러오기
    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 use_fake_hardware:=true robot_ip:=irrelevant launch_rviz:=false

### UR robot moveit 실행
    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5

### 로봇 동작 테스트 파일 실행
    ros2 launch icrs_pick_and_place teamproject_test.py

### 팀프로젝트 수행 파일 실행
    ros2 launch icrs_pick_and_place teamproject_main.py