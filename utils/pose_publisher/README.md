pose_publisher
==============

1. About

    The pose_publisher package contains a ROS node that provides current position and orientation of the robot in the map. This package can be used in multi-robot systems.

2. Nodes

    2.1 pose_publisher
    
    pose_publisher provides current position and orientation of the robot in the map.

    2.1.1 Published Topics
    
    /pose (geometry_msgs/PoseStamped)
    
      Current robot pose. 

    2.1.2 Parameters
    
      ~publish_frequency (double, default: 10.0)
      
        Frequency (hz) at which to publish the robot pose. 

      ~map_frame (std::string, default: map)
      
        The frame attached to the map. 

      ~base_frame (std::string, default: base_link)
      
        The frame attached to the mobile base.
        
3. Usage

    rosrun pose_publisher pose_publisher

4. Related Documentation

    Please refer to the following paper to retrieve more information on the node:

    @inproceedings{yz14simpar,
    author = {Zhi Yan and Luc Fabresse and Jannik Laval and Noury Bouraqadi},
    title = {Team Size Optimization for Multi-robot Exploration},
    booktitle = {In Proceedings of the 4th International Conference on Simulation, Modeling, and Programming for Autonomous Robots (SIMPAR 2014)},
    pages = {438--449},
    address = {Bergamo, Italy},
    month = {October},
    year = {2014}
    }

5. Support

    Zhi Yan, Luc Fabresse, Jannik Laval, and Noury Bouraqadi
    Ecole des Mines de Douai, 59508 Douai, France
    http://car.mines-douai.fr

