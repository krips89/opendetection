.. _basic_structures:

Basic Structures
----------------------------------


The basic classes in OD are Trainers and Detectors. The Trainers and Detectors are grouped under detection methods which we call ObjectDetectors. Even though trainers and detectors can be used 
indepedently, we recommend to use ObjectDetectors as it will prevent the missuse of uncompatible trainers and detectors. 

Each Trainer implements a virtual function `train` with the following signature::

    virtual int train() = 0;

    
And each Detector implements a virtual function `detect` with the following signature::

    virtual int detect(ODScene *scene, std::vector<ODDetection *> &detections) = 0;
    

A very typical code looks like::

      //trainer
      od::ODTrainer *trainer = new od::ODPointCloudGlobalMatchingTrainer(training_input_dir, trained_data_dir);
      trainer->train();

      //detector
      od::ODPointCloudGlobalMatchingDetector<> *detector = new od::ODPointCloudGlobalMatchingDetector<>();
      detector->setTrainingInputLocation(training_input_dir);
      detector->setTrainingDataLocation(trained_data_dir);
      detector->init();

      //Get a scene
      od::ODScenePointCloud<> scene(pointcloud_file);
      vector<od::ODDetection3D *> detections;

      //Detect
      detector->detect(&scene, detections);

      //Do something with the detection
      
For more details please take a look into the examples provided with the `examples` folder in the repository.
