^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_platform
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.7.0 (2025-08-25)
------------------
* Drivetrains (`#250 <https://github.com/clearpathrobotics/clearpath_robot/issues/250>`_)
  * Allow for only 2 motors in fan control node
  * Fixed logic
  * Added 2 joint support for lynx hardware
  * Switch to using motor protection message
* Contributors: Roni Kreinin

2.6.3 (2025-08-18)
------------------

2.6.2 (2025-07-15)
------------------

2.6.1 (2025-07-04)
------------------

2.6.0 (2025-07-04)
------------------
* Lynx 1.0.0 (`#235 <https://github.com/clearpathrobotics/clearpath_robot/issues/235>`_)
  * Multiple boot request attempts
  * Added travel field to feedback
  Use travel for odometry
  * Apply direction to travel
  * Added odometry reset to lynx_motor_driver
  * Record last travel data
  * Last travel in header
  * Renamed service to reset_travel
  * Limit wheel velocity based on system protection max speed
  * Added 1.0.0 bin
* Contributors: Roni Kreinin

2.5.1 (2025-06-17)
------------------
* Populate the A200 MCU status messages (`#224 <https://github.com/clearpathrobotics/clearpath_robot/issues/224>`_)
* Contributors: Chris Iverach-Brereton

2.5.0 (2025-05-29)
------------------
* Renamed to low soc cutoff from low voltage cutoff and fixed service client name.
  * Renamed to low soc cutoff from low voltage cutoff and fixed service client name.
  * Fixed generator member.
* Contributors: Tony Baltovski

2.4.1 (2025-05-20)
------------------
* Muted fan control logs being spammed (`#205 <https://github.com/clearpathrobotics/clearpath_robot/issues/205>`_)
* Contributors: Roni Kreinin

2.4.0 (2025-04-30)
------------------
* Added a software based low voltage cut-off for the A300. (`#185 <https://github.com/clearpathrobotics/clearpath_robot/issues/185>`_)
* Pinout control node (`#198 <https://github.com/clearpathrobotics/clearpath_robot/issues/198>`_)
* Contributors: Roni Kreinin, Tony Baltovski

2.3.3 (2025-04-17)
------------------

2.3.2 (2025-04-16)
------------------

2.3.1 (2025-04-14)
------------------

2.3.0 (2025-04-11)
------------------
* [clearpath_hardware_interfaces] Added user fan control and hysteresis when leaving a warning/error state.
* Jazzy dingo and ridgeback fixes (`#169 <https://github.com/clearpathrobotics/clearpath_robot/issues/169>`_)
* Contributors: Roni Kreinin, Tony Baltovski

2.2.4 (2025-04-07)
------------------

2.2.3 (2025-03-20)
------------------

2.2.2 (2025-03-17)
------------------

2.2.1 (2025-03-12)
------------------

2.2.0 (2025-03-11)
------------------
* Feature/fan diagnostics (`#150 <https://github.com/clearpathrobotics/clearpath_robot/issues/150>`_)
  * Remove trailing spaces
  * Add fan diagnostics
  * Catch accessing indexes beyond the vector size
  * Ensure that abnormal temperatures are always logged with measurement
* Use status full to indicate fully charged batteries (`#148 <https://github.com/clearpathrobotics/clearpath_robot/issues/148>`_)
* Feature/lighting diagnostics (`#144 <https://github.com/clearpathrobotics/clearpath_robot/issues/144>`_)
  * Add lighting diagnostics
  * Remap lighting diagnostic topic
  * Set diagnostic updater hardware id to platform since serial isn't locally available
  * Improve clarity of diagnostic summary text
* Contributors: Hilary Luo, Roni Kreinin

2.1.2 (2025-02-28)
------------------

2.1.1 (2025-02-06)
------------------
* [clearpath_hardware_interfaces] Fixed BMS topic for fan control. (`#145 <https://github.com/clearpathrobotics/clearpath_robot/issues/145>`_)
* Contributors: Tony Baltovski

2.1.0 (2025-01-31)
------------------
* [clearpath_hardware_interfaces] Added license to A300 fan control files. (`#141 <https://github.com/clearpathrobotics/clearpath_robot/issues/141>`_)
* Added initial fan control for A300. (`#136 <https://github.com/clearpathrobotics/clearpath_robot/issues/136>`_)
  * Added initial fan control for A300.
  * Fixed normal command and updated battery range.
* Contributors: Tony Baltovski

2.0.4 (2025-01-22)
------------------

2.0.3 (2025-01-17)
------------------

2.0.2 (2025-01-17)
------------------

2.0.1 (2025-01-17)
------------------

2.0.0 (2025-01-17)
------------------
* Fixed motor error lighting sequence (`#120 <https://github.com/clearpathrobotics/clearpath_robot/issues/120>`_)
  * Update motor error lighting sequence if motor_states changes without the lighting state changing
  Added lighting emulator script
  * Removed light emulator
* 1.1.0
* Changes.
* Add HE2410 & HE2411 battery support (`#119 <https://github.com/clearpathrobotics/clearpath_robot/issues/119>`_) (`#121 <https://github.com/clearpathrobotics/clearpath_robot/issues/121>`_)
  * Cherry-pick from Humble
* Add HE2411 battery support (`#119 <https://github.com/clearpathrobotics/clearpath_robot/issues/119>`_)
  * Add support for the HE2410 and HE2411 batteries
* Add visibility_control.h to clearpath_hardware_interfaces, update includes to reference this file (`#115 <https://github.com/clearpathrobotics/clearpath_robot/issues/115>`_)
* A300 lighting fixes (`#114 <https://github.com/clearpathrobotics/clearpath_robot/issues/114>`_)
  * Set NeedsReset to higher priority than Stopped
  Changed NeedsReset lighting pattern
  Set battery message initial values to not trigger battery fault
  Increased low battery lighting sequence duration
  * Updated low battery sequence
* A300 (`#106 <https://github.com/clearpathrobotics/clearpath_robot/issues/106>`_)
  * Added lynx hardware interface
  * Lynx motor driver
  Rename clearpath_platform namespace to clearpath_hardware_interfaces
  * Added A300 and Inventus battery to generator
  * A300 lighting
  * Dependencies and README
  * Rename platform to hardware_interfaces in hardware.xml
  * Fix append of bms in generator
  * Removed wheel_joints\_ map
  ---------
  Co-authored-by: Luis Camero <lcamero@clearpathrobotics.com>
* Apply twist stamped changes
* Remove platform.launch.py
* Move battery_state to clearpath_hardware_interfaces
* Removed config install
* Renamed header directory
* Add clearpath_hardware_interfaces
* Contributors: Chris Iverach-Brereton, Luis Camero, Roni Kreinin, Tony Baltovski

1.0.1 (2024-11-28)
------------------

1.0.0 (2024-11-26)
------------------
* Fixed version of new package.
* Added minimum version.
* Remove platform.launch.py
* Move battery_state to clearpath_hardware_interfaces
* Removed config install
* Renamed header directory
* Add clearpath_hardware_interfaces
* Contributors: Luis Camero, Tony Baltovski

0.3.4 (2024-10-08)
------------------

0.3.3 (2024-10-04)
------------------

0.3.2 (2024-09-29)
------------------

0.3.1 (2024-09-23)
------------------

0.3.0 (2024-09-19)
------------------
* Changes.
* 0.3.0 Release Candidate with Main Changes (`#81 <https://github.com/clearpathrobotics/clearpath_common/issues/81>`_)
  * Added tests
  * Added action to build from release and source
  * Generator linting erros
  * Customization linting errors
  * Linting
  * Fix: Remove IP address from discovery server launch so it listens on all NICs
  * Changes.
  * 0.2.8
  * Add sysctl config file that changes ipfrag settings to support receiving large messages
  * Added Zed URDF
  * Added Zed to description generator
  * Modified common parameter generation to always flatten
  * Changes.
  * 0.2.9
  * Missing important remapping to mirror hardware topics
  * Added topic to gazebo plugins
  * Updated topic names to match gazebo message types
  * Topics of simulated onboard sensors
  * Realsense adds optical links when in simulator
  * Changes.
  * 0.2.10
  * Modifies platform param to add GQ7 IMU data to ekf_localization and adds GQ7 URDF
  * Fixes styling issues
  * Set spawner as super client
  * Changes.
  * 0.2.11
  * Removed duplicate class
  * Use ROS1 covariance values
  * Updated renamed macanum drive controller
  * Enable gazebo friction plugin on DingoO
  ---------
  Co-authored-by: Hilary Luo <hluo@clearpathrobotics.com>
  Co-authored-by: Tony Baltovski <tbaltovski@clearpathrobotics.com>
  Co-authored-by: Steve Macenski <stevenmacenski@gmail.com>
  Co-authored-by: robbiefish <rob.fisher@hbkworld.com>
* Add headers to Puma hardware
* Updated puma topics
* PumaHardwareInterface
* 0.2.8
* Changes.
* 0.2.7
* Changes.
* 0.2.6
* Changes.
* 0.2.5
* Changes.
* 0.2.4
* Changes.
* Fixed lighting lib install
* 0.2.3
* Changes.
* 0.2.2
* Changes.xx
* Fixed status topic names
* 0.2.1
* Changes.
* Added needs reset lighting pattern
* Contributors: Luis Camero, Roni Kreinin, Tony Baltovski, luis-camero

* Added tests
* Added action to build from release and source
* Generator linting erros
* Customization linting errors
* Linting
* Fix: Remove IP address from discovery server launch so it listens on all NICs
* Add sysctl config file that changes ipfrag settings to support receiving large messages
* Added Zed URDF
* Added Zed to description generator
* Modified common parameter generation to always flatten
* Missing important remapping to mirror hardware topics
* Added topic to gazebo plugins
* Updated topic names to match gazebo message types
* Topics of simulated onboard sensors
* Realsense adds optical links when in simulator
* Modifies platform param to add GQ7 IMU data to ekf_localization and adds GQ7 URDF
* Fixes styling issues
* Set spawner as super client
* Removed duplicate class
* Use ROS1 covariance values
* Updated renamed macanum drive controller
* Enable gazebo friction plugin on DingoO
* Contributors: Luis Camero, Roni Kreinin, Tony Baltovski, luis-camero

0.2.11 (2024-08-08)
-------------------

0.2.10 (2024-07-25)
-------------------

0.2.9 (2024-05-28)
------------------

0.2.8 (2024-05-14)
------------------

0.2.7 (2024-04-08)
------------------

0.2.6 (2024-01-18)
------------------

0.2.5 (2024-01-15)
------------------

0.2.4 (2024-01-11)
------------------
* Fixed lighting lib install
* Contributors: Roni Kreinin

0.2.3 (2024-01-08)
------------------

0.2.2 (2024-01-04)
------------------
* Fixed status topic names
* Contributors: Roni Kreinin

0.2.1 (2023-12-21)
------------------

0.2.0 (2023-12-08)
------------------
* Pass robot description to controller manager over topic
* [clearpath_platform] Re-added position state to hardware interface.
* Added W200 Hardware interface.
* Use path substitution
* Updated lighting patterns
  Added charged state
* Comments
* Cleanup
* Fill lights by platform
* Lighting states
* Working HSV
* Initial lighting node
* Whitespace
* Base diff drive hardware and hardware interface class
  J100 and W200 inherit from diff drive
  Moved each platform into its own folder
* Contributors: Luis Camero, Roni Kreinin, Tony Baltovski

0.1.3 (2023-11-03)
------------------

0.1.2 (2023-10-02)
------------------

0.1.1 (2023-08-25)
------------------

0.1.0 (2023-08-17)
------------------

0.0.9 (2023-07-31)
------------------

0.0.8 (2023-07-24)
------------------

0.0.7 (2023-07-19)
------------------

0.0.6 (2023-07-13)
------------------

0.0.5 (2023-07-12)
------------------

0.0.4 (2023-07-07)
------------------

0.0.3 (2023-07-05)
------------------

0.0.2 (2023-07-04)
------------------

0.0.1 (2023-06-21)
------------------
* Added namespacing support
* Updated dependencies
* Added clearpath_generator_common
  Moved clearpath_platform to clearpath_common
  Fixed use_sim_time parameter issue with ekf_node
* Contributors: Roni Kreinin
