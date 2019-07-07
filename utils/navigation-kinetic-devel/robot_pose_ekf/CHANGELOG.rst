^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_pose_ekf
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.12.13 (2016-08-15)
--------------------

1.12.12 (2016-06-24)
--------------------

1.12.11 (2016-06-08)
--------------------

1.12.10 (2016-05-27)
--------------------

1.12.9 (2016-05-26)
-------------------

1.12.8 (2016-05-16)
-------------------

1.12.7 (2016-01-05)
-------------------

1.12.6 (2016-01-02)
-------------------

1.12.5 (2015-10-29)
-------------------

1.12.4 (2015-06-03)
-------------------

1.12.3 (2015-04-30)
-------------------

1.12.2 (2015-03-31)
-------------------

1.12.1 (2015-03-14)
-------------------

1.12.0 (2015-02-04)
-------------------
* update maintainer email
* Contributors: Michael Ferguson

1.11.15 (2015-02-03)
--------------------
* Fix bfl includes in robot_pose_ekf
* Contributors: Jochen Sprickerhof

1.11.14 (2014-12-05)
--------------------

1.11.13 (2014-10-02)
--------------------

1.11.12 (2014-10-01)
--------------------
* Fix EKF topic name so that it is unaffected by tf_prefix
* Install launch files, closes `#249 <https://github.com/ros-planning/navigation/issues/249>`_
* Fixed hardcoded tf frames issue by fetching base_footprint_frame_ value from param server and using it in OdomEstimation filter
* Fixed hardcoded tf frames issue by adding variables for output and base_footprint frames along with mutator methods
* Contributors: Jochen Sprickerhof, Michael Ferguson, Murilo FM

1.11.11 (2014-07-23)
--------------------

1.11.10 (2014-06-25)
--------------------

1.11.9 (2014-06-10)
-------------------
* fix robot_pose_ekf test
* Contributors: Michael Ferguson

1.11.8 (2014-05-21)
-------------------
* fix build, was broken by `#175 <https://github.com/ros-planning/navigation/issues/175>`_
* Contributors: Michael Ferguson

1.11.7 (2014-05-21)
-------------------
* Even longer Time limit for EKF Test
* make rostest in CMakeLists optional
* Contributors: David Lu!!, Lukas Bulwahn

1.11.5 (2014-01-30)
-------------------
* check for CATKIN_ENABLE_TESTING
* Download test data from download.ros.org instead of willow
* Change maintainer from Hersh to Lu

1.11.4 (2013-09-27)
-------------------
* Package URL Updates
* upgrade depracated download data calls.
* use tf_prefix to lookup and send transforms
