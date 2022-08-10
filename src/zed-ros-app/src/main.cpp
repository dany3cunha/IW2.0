#include "headers.hpp"

void pointCloud2_callback(const sensor_msgs::PointCloud2 &ptCloud)
{

    for (sensor_msgs::PointCloud2ConstIterator<float> it(ptCloud, "x"); it != it.end(); ++it)
    {
        // TODO: do something with the values of x, y, z
        //std::cout << it[0] << ", " << it[1] << ", " << it[2] << '\n';
    }
}
/**
 * Node main function
 */
int main(int argc, char **argv)
{
#if 1
    auto streaming_devices = Camera::getStreamingDeviceList();
    int nb_streaming_zed = streaming_devices.size();

    cout << "Detect: " + to_string(nb_streaming_zed) + " ZED in streaming" << endl;
    if (nb_streaming_zed == 0)
    {
        cout << "No streaming ZED detected, have you take a look to the sample 'ZED Streaming Sender' ?" << endl;
        return 0;
    }

    for (auto &it : streaming_devices)
        cout << "* ZED: " << it.serial_number << ", IP: " << it.ip << ", port : " << it.port << ", bitrate : " << it.current_bitrate << "\n";
#endif

    Camera zed;
    // Setup configuration parameters for the ZED
    InitParameters init_parameters;
    init_parameters.coordinate_units = UNIT::METER;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD; // This system is for ROS

    init_parameters.input.setFromStream("127.0.0.1", 5000);

    // Open the camera
    ERROR_CODE zed_open_state = zed.open(init_parameters);
    if (zed_open_state != ERROR_CODE::SUCCESS)
    {
        cout << "Camera Open" << zed_open_state << "Exit program." << endl;
        return EXIT_FAILURE;
    }

    auto camera_info = zed.getCameraInformation();
    cout << endl;
    cout << "ZED Camera Resolution     : " << camera_info.camera_configuration.resolution.width << "x" << camera_info.camera_configuration.resolution.height << endl;
    cout << "ZED Camera FPS            : " << zed.getInitParameters().camera_fps << endl;

    Mat image;   // current left image
    Pose pose;   // positional tracking data
    Plane plane; // detected plane
    Mesh mesh;   // plane mesh

    ERROR_CODE find_plane_status = ERROR_CODE::SUCCESS;
    POSITIONAL_TRACKING_STATE tracking_state = POSITIONAL_TRACKING_STATE::OFF;

    // time stamp of the last mesh request
    chrono::high_resolution_clock::time_point ts_last;

    sl::PositionalTrackingParameters tracking_parameters = PositionalTrackingParameters();
    tracking_parameters.enable_pose_smoothing = true;

    // Enable positional tracking before starting spatial mapping
    zed.enablePositionalTracking(tracking_parameters);

    RuntimeParameters runtime_parameters;
    runtime_parameters.measure3D_reference_frame = REFERENCE_FRAME::CAMERA;

    // Prepare new image size to retrieve half-resolution images
    Resolution image_size = zed.getCameraInformation().camera_resolution;
    int new_width = image_size.width / 2;
    int new_height = image_size.height / 2;
    Resolution new_image_size(new_width, new_height);

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    sl::Mat image_zed(new_width, new_height, MAT_TYPE::U8_C4);
    cv::Mat image_ocv = slMat2cvMat(image_zed);

    ros::init(argc, argv, "zed_ros_floor_detection");
    ros::NodeHandle n1, n2, cmd_config;
    ros::NodeHandle pointCloud2_node;

    ros::Publisher floor_PubMarker = n1.advertise<visualization_msgs::Marker>("/zed2/zed_node/plane_marker", 1, false);
    // ros::Publisher floor_PubPlane = n2.advertise<zed_interfaces::PlaneStamped>("zed2/zed_node/plane", 10, false);
    ros::Publisher cmd_ConfigPub = cmd_config.advertise<std_msgs::String>("/zed2_cmd_config", 1, false);

    ros::Subscriber ptCloud_sub = pointCloud2_node.subscribe("/zed2/zed_node/point_cloud/cloud_registered", 10, pointCloud2_callback);

    ros::Rate loop_rate(ROS_loopRate);

    std_msgs::String msg;

    int exposure = open_exposure;
    while (ros::ok())
    {
        int old_exposure = exposure;
        adjustCameraExposure(image_ocv, exposure);
        if (exposure != old_exposure)
        {
            msg.data = to_string((int)sl::VIDEO_SETTINGS::EXPOSURE) + "," + to_string(exposure);
            cmd_ConfigPub.publish(msg);
            cout << "EXP: " << exposure << endl;
        }

        if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS)
        {
            // Retrieve image in GPU memory
            if (zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU, new_image_size) == ERROR_CODE::SUCCESS)
            {
                // Update pose data (used for projection of the mesh over the current image)
                tracking_state = zed.getPosition(pose);

                if (tracking_state == POSITIONAL_TRACKING_STATE::OK)
                {
                    // Compute elapse time since the last call of plane detection
                    auto duration = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - ts_last).count();

                    Transform resetTrackingFloorFrame;
                    find_plane_status = zed.findFloorPlane(plane, resetTrackingFloorFrame);

                    if (find_plane_status == ERROR_CODE::SUCCESS && duration > 500)
                    {

                        mesh = plane.extractMesh();

                        visualization_msgs::MarkerPtr plane_marker = boost::make_shared<visualization_msgs::Marker>();
                        meshToPlaneMarker(plane_marker, mesh, pose);
                        // Publish the marker
                        floor_PubMarker.publish(plane_marker);
                        //  <---- Publish the plane as green mesh

                        sl::float3 vector_normal = plane.getNormal();
                        sl::float4 eq = plane.getPlaneEquation();
                        cout << "Floor normal(x,y,z) " << vector_normal.x << " " << vector_normal.y << " " << vector_normal.z << endl;
                        cout << "Floor plane ax+by+cz=d " << eq.x << " " << eq.y << " " << eq.z << " " << eq.w << endl;

                        // zed_interfaces::PlaneStampedPtr planeMsg = boost::make_shared<zed_interfaces::PlaneStamped>();
                        //  planeAsCustomMessage(planeMsg, plane);
                        //   Publish custom message
                        //   floor_PubPlane.publish(planeMsg);
                        //   <---- Publish plane as custom message
                        ts_last = chrono::high_resolution_clock::now();
                    }
                    else
                    {
                        mesh.clear();
                    }
                }
            }
        }

        if (!ros::master::check())
        {
            ros::shutdown();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    zed.disablePositionalTracking();
    zed.close();

    return 0;
}

void adjustCameraExposure(cv::Mat cv_image, int &exposure)
{

    cv::Mat hsv;
    cv::cvtColor(cv_image, hsv, cv::COLOR_BGR2HSV);

    int sum = 0;

    for (int i = 0; i < hsv.rows; i++)
    {
        for (int j = 0; j < hsv.cols; j++)
        {
            cv::Vec3b HSV = hsv.at<cv::Vec3b>(i, j);
            if (HSV.val[2] > 200)
                sum++;
        }
    }
    double x = 100 * double(sum) / (double(hsv.rows) * double(hsv.cols));
    cout << "HSV: " << x << " %" << endl;

    if (x > maxExposure_thres)
    {
        if (exposure > 1)
        {
            exposure--;
        }
        else
        {
            exposure = 0;
        }
    }
    if (x < minExposure_thres)
    {
        exposure++;
    }

    cv::waitKey(10);
}

void meshToPlaneMarker(visualization_msgs::MarkerPtr &plane_marker, sl::Mesh mesh, sl::Pose pose)
{

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    static int plane_mesh_id = 0;
    ros::Time ts = ros::Time::now();
    plane_marker->header.stamp = ts;
    // Set the marker action.  Options are ADD and DELETE
    plane_marker->action = visualization_msgs::Marker::ADD;
    plane_marker->lifetime = ros::Duration(planeDuration);

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    plane_marker->ns = "plane_meshes";
    plane_marker->id = plane_mesh_id++;
    plane_marker->header.frame_id = "zed2_left_camera_frame";

    // Set the marker type.
    plane_marker->type = visualization_msgs::Marker::TRIANGLE_LIST;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    plane_marker->pose.position.x = 0;
    plane_marker->pose.position.y = 0;
    plane_marker->pose.position.z = 0;
    plane_marker->pose.orientation.x = 0;
    plane_marker->pose.orientation.y = 0;
    plane_marker->pose.orientation.z = 0;
    plane_marker->pose.orientation.w = 1;

    // Set the color -- be sure to set alpha to something non-zero!
    plane_marker->color.r = 0.10f;
    plane_marker->color.g = 0.75f;
    plane_marker->color.b = 0.20f;
    plane_marker->color.a = 0.7;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    plane_marker->scale.x = 1.0;
    plane_marker->scale.y = 1.0;
    plane_marker->scale.z = 1.0;

    size_t triangCount = mesh.getNumberOfTriangles();
    size_t ptCount = triangCount * 3;
    plane_marker->points.resize(ptCount);
    plane_marker->colors.resize(ptCount);

    size_t ptIdx = 0;
    for (size_t t = 0; t < triangCount; t++)
    {
        for (int p = 0; p < 3; p++)
        {

            uint vIdx = mesh.triangles[t][p];
            plane_marker->points[ptIdx].x = mesh.vertices[vIdx][0];
            plane_marker->points[ptIdx].y = mesh.vertices[vIdx][1];
            plane_marker->points[ptIdx].z = mesh.vertices[vIdx][2];

            // Set the color -- be sure to set alpha to something non-zero!
            plane_marker->colors[ptIdx].r = 0.10f;
            plane_marker->colors[ptIdx].g = 0.75f;
            plane_marker->colors[ptIdx].b = 0.20f;
            plane_marker->colors[ptIdx].a = 0.7;

            ptIdx++;
        }
    }
}

void planeAsCustomMessage(zed_interfaces::PlaneStampedPtr &planeMsg, sl::Plane plane)
{

    ros::Time ts = ros::Time::now();
    planeMsg->header.stamp = ts;
    planeMsg->header.frame_id = "zed2_left_camera_frame";

    // Plane equation
    sl::float4 sl_coeff = plane.getPlaneEquation();
    planeMsg->coefficients.coef[0] = static_cast<double>(sl_coeff[0]);
    planeMsg->coefficients.coef[1] = static_cast<double>(sl_coeff[1]);
    planeMsg->coefficients.coef[2] = static_cast<double>(sl_coeff[2]);
    planeMsg->coefficients.coef[3] = static_cast<double>(sl_coeff[3]);

    // Plane Normal
    sl::float3 sl_normal = plane.getNormal();
    planeMsg->normal.x = sl_normal[0];
    planeMsg->normal.y = sl_normal[1];
    planeMsg->normal.z = sl_normal[2];

    // Plane Center
    sl::float3 sl_center = plane.getCenter();
    planeMsg->center.x = sl_center[0];
    planeMsg->center.y = sl_center[1];
    planeMsg->center.z = sl_center[2];

    // Plane extents
    sl::float3 sl_extents = plane.getExtents();
    planeMsg->extents[0] = sl_extents[0];
    planeMsg->extents[1] = sl_extents[1];

    // Plane pose
    sl::Pose sl_pose = plane.getPose();
    sl::Orientation sl_rot = sl_pose.getOrientation();
    sl::Translation sl_tr = sl_pose.getTranslation();

    planeMsg->pose.rotation.x = sl_rot.ox;
    planeMsg->pose.rotation.y = sl_rot.oy;
    planeMsg->pose.rotation.z = sl_rot.oz;
    planeMsg->pose.rotation.w = sl_rot.ow;

    planeMsg->pose.translation.x = sl_tr.x;
    planeMsg->pose.translation.y = sl_tr.y;
    planeMsg->pose.translation.z = sl_tr.z;

    // Plane Bounds
    std::vector<sl::float3> sl_bounds = plane.getBounds();
    planeMsg->bounds.points.resize(sl_bounds.size());
    memcpy(planeMsg->bounds.points.data(), sl_bounds.data(), 3 * sl_bounds.size() * sizeof(float));

    // Plane mesh
    sl::Mesh sl_mesh = plane.extractMesh();
    size_t triangCount = sl_mesh.triangles.size();
    size_t ptsCount = sl_mesh.vertices.size();
    planeMsg->mesh.triangles.resize(triangCount);
    planeMsg->mesh.vertices.resize(ptsCount);

    // memcpy not allowed because data types are different
    for (size_t i = 0; i < triangCount; i++)
    {
        planeMsg->mesh.triangles[i].vertex_indices[0] = sl_mesh.triangles[i][0];
        planeMsg->mesh.triangles[i].vertex_indices[1] = sl_mesh.triangles[i][1];
        planeMsg->mesh.triangles[i].vertex_indices[2] = sl_mesh.triangles[i][2];
    }

    // memcpy not allowed because data types are different
    for (size_t i = 0; i < ptsCount; i++)
    {
        planeMsg->mesh.vertices[i].x = sl_mesh.vertices[i][0];
        planeMsg->mesh.vertices[i].y = sl_mesh.vertices[i][1];
        planeMsg->mesh.vertices[i].z = sl_mesh.vertices[i][2];
    }
}

cv::Mat slMat2cvMat(sl::Mat &input)
{

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(sl::MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}

int getOCVtype(sl::MAT_TYPE type)
{
    int cv_type = -1;
    switch (type)
    {
    case sl::MAT_TYPE::F32_C1:
        cv_type = CV_32FC1;
        break;
    case sl::MAT_TYPE::F32_C2:
        cv_type = CV_32FC2;
        break;
    case sl::MAT_TYPE::F32_C3:
        cv_type = CV_32FC3;
        break;
    case sl::MAT_TYPE::F32_C4:
        cv_type = CV_32FC4;
        break;
    case sl::MAT_TYPE::U8_C1:
        cv_type = CV_8UC1;
        break;
    case sl::MAT_TYPE::U8_C2:
        cv_type = CV_8UC2;
        break;
    case sl::MAT_TYPE::U8_C3:
        cv_type = CV_8UC3;
        break;
    case sl::MAT_TYPE::U8_C4:
        cv_type = CV_8UC4;
        break;
    default:
        break;
    }
    return cv_type;
}

void planesVectors(sl::Plane plane, sl::Camera zed)
{
    sl::float3 vector_normal = plane.getNormal();
    sl::float4 eq = plane.getPlaneEquation();
    SensorsData sensors_data;
    SensorsData::IMUData imu_data;

    zed.getSensorsData(sensors_data, TIME_REFERENCE::IMAGE); // Retrieve only frame synchronized data

    // Extract IMU data
    imu_data = sensors_data.imu;

    // Retrieve linear acceleration and angular velocity
    sl::float3 linear_acceleration = imu_data.linear_acceleration;

    cout << "Camera normal(x,y,z) " << linear_acceleration.x << " " << linear_acceleration.y << " " << linear_acceleration.z << endl;
    cout << "Floor normal(x,y,z) " << vector_normal.x << " " << vector_normal.y << " " << vector_normal.z << endl;
    cout << "Floor plane ax+by+cz=d " << eq.x << " " << eq.y << " " << eq.z << " " << eq.w << endl;
}
