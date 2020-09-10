# Turtlesim Simple Planner

<details open>
<summary> <b>Brief Review<b></summary>
Note that the preffered language of this tutorial is using 
<img src="https://img.shields.io/badge/-C++-0390fc?style=for-the-badge&logo=c&logoColor=white&labelColor=4B8BBE" />

<br></br>
<p align="center">
<img src = "https://github.com/issaiass/Turtlesim_Simple_Planner/blob/master/imgs/planner.GIF?raw=true" width="60%"/>
</p>
<br></br>

The project tree:

<br></br>
<p align="center">
<img src = "https://github.com/issaiass/Turtlesim_Simple_Planner/blob/master/imgs/tree.PNG?raw=true" width="50%"/>
</p>
<br></br>

The rqt graph (3 turtles for simplicity):

<br></br>
<p align="center">
<img src = "https://github.com/issaiass/Turtlesim_Simple_Planner/blob/master/imgs/simple_planner_3turtles_graph.PNG?raw=true" width="50%"/>
</p>
<br></br>

The tf tree (3 turtles for simplicity):

<br></br>
<p align="center">
<img src = "https://github.com/issaiass/Turtlesim_Simple_Planner/blob/master/imgs/simple_planner_3turtles_tf.PNG?raw=true" width="50%"/>
</p>
<br></br>

A turtlesim program that will try to avoid each turtle in the field. 

This programs will help you continuing to understand the basic ROS concepts like tf2, also I include how to make libraries and classes to be more organized with your code.

This applications function as follows.
- ROSCore launches
- Turtlesim node launches and a default turtle is displayed at the center. 
- Next we lauched three (3) additional nodes...
  - The first node will broadcast using tf2 the goal of the turtles. These goals are static broadcasted to */tf_static* topic.
  - The second additional node is a pose broadcaster.  Those poses are also broadcasted on the */tf* topic.
  - The final node is the controller.  This is a *simple planner* application.
- Next the turtles tries to find its path to the new locations and will try to avoid each one with a minimal distance margin of acceptance.
- Each planner uses a *PID controller* for each turtle and also there is another part of the code responsible of the *avoidance algorithm* (naive approach).

**If you do not know the ros basics i recommend my previous package named  *TurtlesimBuster*, but if you have now more ROS experience just continue with the section below**

Please follow [ROS tf](http://wiki.ros.org/tf/) and [tf2](http://wiki.ros.org/tf/2) tutorials and also [this one from the SIR group of UPC Spain](https://sir.upc.edu/projects/rostutorials/5-client-server_tutorial/index.html) for better understanding of Rigid Body Transformations.

</details>

<details open>
<summary> <b>Using TurtlesimBuster Package<b></summary>

- Create a ROS ros workspace and compile an empty package:
~~~
    cd ~
    mkdir -p catkin_ws/src
    cd catkin_ws
    catkin_make
~~~
- Open the `.bashrc` with nano:
~~~
    nano ~/.bashrc
~~~    
- Insert this line at the end of the `~/.bashrc` file for sourcing your workspace:
~~~
    source ~/catkin_ws/devel/setup.bash
~~~
- Clone this repo in the `~/catkin_ws/src` folder by typing:
~~~ 
    cd ~/catkin_ws/src
    git clone https://github.com/issaiass/turtlesim_simple_planner
    mv Turtlesim_Simple_Planner turtlesim_simple_planner
    rm -rf README.md
    rm -rf imgs
~~~
- Go to the root folder `~/catkin_ws` and make the folder running `catkin_make` to ensure the application compiles.
- Finally launch the application by:
~~~
    roslaunch turtlesim_simple_planner simple_planner.launch
~~~
- You must see that `roscore` and `turtlesim_node` executing.  In the end eight (8) turtles are spawned (+1 static defaul turtle) trying not to overlap each other.

<details close>
<summary> <b>Modifying the Package for your own usage<b></summary>
NOTE: The node names must not be similar.

After completing the previous steps and ensuring the application is compiling do the following:

- Open the launch file at the location `~/catkin_ws/src/turtlesim_simple_planer/launch`.
- Modify the node planner by creating a turtle in a location on the simulator:
~~~
    <node name="planner_w" pkg="turtlesim_simple_planner" type="simple_planner" output="screen"
        args="<turtle_name> <x0> <y0> <theta>">
  </node> 
~~~
- Modify the pose broadcaster, note the `turtle_name` is the same name as previous step:
~~~
    <node name="turtle_w_broadcaster" pkg="turtlesim_simple_planner" type="pose_broadcaster" respawn="false" output="screen" >
    <param name="sdturtle" type="string" value="<turtle_name>" />
  </node>
~~~
- Modify the frame broadcasters, note that `turtle_name` at the end has the word `_goal`
- Modify the locations
~~~
    <node name="turtle_w_static_goal" pkg="turtlesim_simple_planner" type="goal_broadcaster" args="<turtle_name>_goal <x1> <y1> <z1> <roll> <pitch> <yaw>" />  
~~~
- Open the file `simple_planner.cpp` and hardcode the line 84 with all your turtle names plus the fixed one, named `turtle1_fixed`, i.e.:
~~~
    vector<string> turtles{"turtle_lt", "turtle_rt", "turtle_up", "turtle_dn", "turtle1_fixed", "turtle_x", "turtle_y", "turtle_z", "turtle_w"};   
~~~
- Finally modify line 7 to line 23 as your choise to tune the application.
- Go to the root folder `~/catkin_ws` and make the folder running `catkin_make` to ensure the application compiles.
- Finally launch the application by:
~~~
    roslaunch turtlesim_simple_planner simple_planner.launch
~~~
</details>

</details>

<details open>
<summary> <b>Results<b></summary>

You could see the results on this youtube video.  

<p align="center">

[<img src= "https://img.youtube.com/vi/1B2E3xWYfeI/0.jpg" />](https://youtu.be/1B2E3xWYfeI)

</p>

The video only shows the application running, not the explanation of the code.


</details>

<details open>
<summary> <b>Video Explanation<b></summary>

I will try my best for making an explanatory video of the application as in this youtube video.

<p align="center">

[<img src= "" />]()

</p>

</details>

<details open>
<summary> <b>Issues<b></summary>

:bug: The kill function is not working at all but is not needed for running the application.

</details>

<details open>
<summary> <b>Contributiong<b></summary>

Your contributions are always welcome! Please feel free to fork and modify the content but remember to finally do a pull request.

</details>

<details open>
<summary> :iphone: <b>Having Problems?<b></summary>

<p align = "center">

[<img src="https://img.shields.io/badge/linkedin-%230077B5.svg?&style=for-the-badge&logo=linkedin&logoColor=white" />](https://www.linkedin.com/in/riawa)
[<img src="https://img.shields.io/badge/telegram-2CA5E0?style=for-the-badge&logo=telegram&logoColor=white"/>](https://t.me/issaiass)
[<img src="https://img.shields.io/badge/instagram-%23E4405F.svg?&style=for-the-badge&logo=instagram&logoColor=white">](https://www.instagram.com/daqsyspty/)
[<img src="https://img.shields.io/badge/twitter-%231DA1F2.svg?&style=for-the-badge&logo=twitter&logoColor=white" />](https://twitter.com/daqsyspty) 
[<img src ="https://img.shields.io/badge/facebook-%233b5998.svg?&style=for-the-badge&logo=facebook&logoColor=white%22">](https://www.facebook.com/daqsyspty)
[<img src="https://img.shields.io/badge/linkedin-%230077B5.svg?&style=for-the-badge&logo=linkedin&logoColor=white" />](https://www.linkedin.com/in/riawe)
[<img src="https://img.shields.io/badge/tiktok-%23000000.svg?&style=for-the-badge&logo=tiktok&logoColor=white" />](https://www.linkedin.com/in/riawe)
[<img src="https://img.shields.io/badge/whatsapp-%23075e54.svg?&style=for-the-badge&logo=whatsapp&logoColor=white" />](https://wa.me/50766168542?text=Hello%20Rangel)
[<img src="https://img.shields.io/badge/hotmail-%23ffbb00.svg?&style=for-the-badge&logo=hotmail&logoColor=white" />](mailto:issaiass@hotmail.com)
[<img src="https://img.shields.io/badge/gmail-%23D14836.svg?&style=for-the-badge&logo=gmail&logoColor=white" />](mailto:riawalles@gmail.com)

</p

</details>

<details open>
<summary> <b>License<b></summary>
<p align = "center">
<img src= "https://mirrors.creativecommons.org/presskit/buttons/88x31/svg/by-sa.svg" />
</p>
</details>
