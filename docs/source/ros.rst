
Requirements
============

Install ros2 (humble) 
---------------------

Follow the steps in the official documentation : `ros_doc`_

Install the dev-tools and any required ros-package for your setup.

Source :
.. code-block:: bash   

    source /opt/ros/humble/setup.bash

You can also add this line to the *.bashrc* file, to avoid repeting it every time you open a terminal.

.. _ros_doc: https://docs.ros.org/en/humble/Installation.html

Install python modules
----------------------

.. code-block:: bash  
    pip3 install -r requirements.txt

Depending on your setup you may prefer to install within a virtual environment.

Adapt to your setup:
====================

Data organisation 
-----------------

You can send one video, images (png or jpg format!) and csv files containing proprioceptive data. You can choose to send just a video, or just images, or just proprioceptive data, or any combination of them.

You will have to organise your data as follow, depending on which type of data you want to use; **keep the folders names and files extensions!** :

.. code-block:: bash  
    -- root/
        -- video/
            {name}.mp4
        -- csv_files/
            [{name}.csv]
        -- images/
            [{name}.png]
        -- converted_time_series (optional)
            [{name}.png]

You can also add proprioception already converted to images as an option, instead of using csv files or as an addition to csv files.

Conversion script
-----------------

At **identification/identification/convert_csv_to_image** create your script **convert_{id}.py** with a desired id, to convert proprioceptive data into images. Do it given your csv structure and the data you want to use and the physical details you want to capture. You can have a look at the existant script to get started. If you don't want to go through csv files, you can ignore this step and either already structure converted_time_series/ folder or not use proprioception at all.

Prompts
-------

At **identification/identification/prompts** create your script **prompts_{id}.py** with a desired id, containing your system prompt and 2 user_prompts, defined as SYSTEM_PROMPT, USER_PROMPT1 and USER_PROMPT2.

Templates are given for each :

.. code-block:: bash  
    SYSTEM_PROMPT="""
    [SYSTEM]:
    You are a ...

    [TASKS]:
    ...

    [MANIPULATED OBJECTS]:
    -
    -
    ...

    [DATA INPUT]:
    The data you will receive is:
    -
    -
    ...

    [KNOWN CORRELATIONS]:
    -
    -
    ...

    [DECISION RULES]:
    -
    -
    ...

    [Actions]:
    1.
    2.
    ...

    [OUTPUT FORMAT]:
    <JSON EMPTY STRUCTURE>
    """

    USER_PROMPT1="""
    You are provided with a batch of data
    corresponding to one robot action execution.
    ...
    Follow these reasoning steps:
    1. ...
    2. ...
    ...

    Requirements:
    ...

    Reminder:
    -
    -
    ...
    """

    USER_PROMPT2="""
    You will now be given
    several classification examples...

    --- Classification Examples ---

    -- Example 1:

    Analysis:
    <ANALYSIS>
    <POPULATED JSON STRUCTURE>
    Correct action:
    <CORRESPONDING ACTION>

    -- Example 2:
    ...

    --- End of Examples ---
    Now classify YOUR previous JSON
    into one action from the system prompt.

    Format:
    <CORRESPONDING ACTION>
    """

Add your id in the maps
-----------------------

Within **identification/identification/mappings.py** add your prompt pointer, conversion pointer (if any) and actions classes under your chosen id, within related mappings.

Define parameters
-----------------

Define your setup parameters at **identification/config/parameters.yaml** :

**data_root_path** : The root to all your data folders as defined [above](#data-organisation)

**id** : The chosen id for you setup

Test on an example 
==================

By default the configuration is set to run on the provided example **data_sample_example/**.
You can perform a test as follow:
1. Open a terminal at the root and build the project :
.. code-block:: bash  
    colcon build --symlink-install

2. Source :
.. code-block:: bash  
    source install/setup.bash
3. Launch the node :
.. code-block:: bash  
    ros2 launch identification inference.launch.py
4. Open another terminal, source again, and launch the action_info topic:
.. code-block:: bash  
    ros2 topic echo /action_info
5. Open another terminal, source again, and call client:
.. code-block:: bash  
    ros2 service call /action_type identification_interface/srv/ActionType "{action_type_request: true}"

You should be able to see the classfication of the action from /action_info topic, after some seconds.

You can also play with the content of the example, by removing or adding some modalities. As long as you keep at least one folder from the data structure.

Include to your project 
=======================

Add the repository as a submodule 

.. code-block:: bash  
    git submodule add https://gitlab.lri.cea.fr/ROS_developers/resources/VLM_local_action_classification.git

    git submodule init

    git submodule update

Send inference request 
----------------------

Within your superproject, use the service definition at **identification/include/identification/action_type_client.hpp**, to send a request of action classification to the VLM. The service interface is defined within **identification_interface/**.

Add launch file to your desired launch instance
-----------------------------------------------

You will find the launch file at **identification/launch**.

Build your project 
------------------

.. code-block:: bash  
    colcon build --symlink_install

Visualize publisher info
------------------------

Once you execute your project, you can visualize the actions info like this :

.. code-block:: bash 
    ros2 topic echo /action_info
