This classifier relies on object shape and is based on the approach of *[W. Wohlkinger and M. Vincze, Ensemble of Shape Functions for 3D Object Classification]*.

It uses CAD data for training, such as https://repo.acin.tuwien.ac.at/tmp/permanent/Cat200_ModelDatabase.zip.

In order to speed up training, please select a subset of the training data you want to use and put it in a folder `Cat200_ModelDatabase__small`.

The classifier can then be started by:

`roslaunch openni_launch openni.launch depth_registration:=true`
`roslaunch object_classifier classifier_demo.launch models_dir:=your_dataset_dir/Cat200_ModelDatabase__small/ topic:=/camera/depth_registered/points training_dir:=your_training_dir`

, where *models_dir* is your training directory with the CAD models of the classes, *training_dir* is the directory containing the trained data (if they exist - otherwise they will be re-trained) and *topic* is the camera topic.

The launch file also lets you choose which segmentation type you want to use. The *pcl_object_segmenter* segments objects on a table plane, whereby the highest plane parallel to the largest plane (usually the floor) is considered as table.
On the contrary, the *object_rgb_segmenter* segments all objects within a distance *chop_z* from the camera. The segmentation takes a bit longer in this case.

