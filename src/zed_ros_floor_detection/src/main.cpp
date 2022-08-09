#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <zed_interfaces/PlaneStamped.h>

// Standard includes
#include <stdio.h>
#include <string.h>

// ZED includes
#include <sl/Camera.hpp>

// Using std and sl namespaces
using namespace std;
using namespace sl;

void coincedence(Mesh one, Mesh two);
void findSimilarTriangle(uint &n, sl::float3 vertices, Mesh haystack);

void parseArgs(int argc, char **argv, sl::InitParameters &param);
void meshToPlaneMarker(visualization_msgs::MarkerPtr &plane_marker, sl::Mesh mesh, sl::Pose pose);
void planeAsCustomMessage(zed_interfaces::PlaneStampedPtr &planeMsg, sl::Plane plane);

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

    // init_parameters.depth_mode = DEPTH_MODE::ULTRA; // The default is ULTRA! If jetson, is PERFORMANCE

    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD; // This system is for ROS
    //init_parameters.sdk_verbose = true;

    parseArgs(argc, argv, init_parameters);
    /*
    string stream_params;
    if (argc > 1)
    {
        stream_params = string(argv[1]);
    }
    else
    {
        cout << "\nOpening the stream requires the IP of the sender\n";
        cout << "Usage : ./ZED_Streaming_Receiver IP:[port]\n";
        cout << "You can specify it now, then press ENTER, 'IP:[port]': ";
        cin >> stream_params;
    }

    setStreamParameter(init_parameters, stream_params);
    */
    // Open the camera
    ERROR_CODE zed_open_state = zed.open(init_parameters);
    if (zed_open_state != ERROR_CODE::SUCCESS)
    {
        cout << "Camera Open" << zed_open_state << "Exit program." << endl;
        return EXIT_FAILURE;
    }

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

    int i = 0;
    Mesh one, two;

    ros::init(argc, argv, "zed_ros_floor_detection");

    ros::NodeHandle n1;
    ros::NodeHandle n2;

    ros::Publisher floor_PubMarker = n1.advertise<visualization_msgs::Marker>("zed2/zed_node/plane_marker", 10, false);
    // ros::Publisher floor_PubPlane = n2.advertise<zed_interfaces::PlaneStamped>("zed2/zed_node/plane", 10, false);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS)
        {
            // Retrieve image in GPU memory
            zed.retrieveImage(image, VIEW::LEFT, MEM::GPU);
            // Update pose data (used for projection of the mesh over the current image)
            tracking_state = zed.getPosition(pose);

            if (tracking_state == POSITIONAL_TRACKING_STATE::OK)
            {
                // Compute elapse time since the last call of plane detection
                auto duration = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - ts_last).count();

                // if 500ms have spend since last request
                if (true /*(duration > 500)*/ /* && user_action.press_space*/)
                {
                    // Update pose data (used for projection of the mesh over the current image)
                    Transform resetTrackingFloorFrame;
                    find_plane_status = zed.findFloorPlane(plane, resetTrackingFloorFrame);
                    ts_last = chrono::high_resolution_clock::now();
                }

                if (find_plane_status == ERROR_CODE::SUCCESS)
                {
                    /* Higher the filter, more errors occur
                    sl::MeshFilterParameters meshFilter;
                    meshFilter.set(MeshFilterParameters::MESH_FILTER::HIGH);
                    mesh.filter(meshFilter);
                    */

                    mesh = plane.extractMesh();
                    /*
                    if (i == 0)
                    {
                        if (mesh.getNumberOfTriangles() != 0)
                        {
                            one = mesh;
                            i++;
                        }
                    }

                    else if (i == 1)
                    {
                        if (mesh.getNumberOfTriangles() != 0)
                        {
                            two = mesh;
                            i++;
                        }
                    }
                    */
                    visualization_msgs::MarkerPtr plane_marker = boost::make_shared<visualization_msgs::Marker>();
                    meshToPlaneMarker(plane_marker, mesh, pose);
                    
                    // Publish the marker
                    floor_PubMarker.publish(plane_marker);
                    //  <---- Publish the plane as green mesh

                    zed_interfaces::PlaneStampedPtr planeMsg = boost::make_shared<zed_interfaces::PlaneStamped>();
                    // planeAsCustomMessage(planeMsg, plane);
                    //  Publish custom message
                    //  floor_PubPlane.publish(planeMsg);
                    //  <---- Publish plane as custom message
                }
                else /*if (find_plane_status == ERROR_CODE::PLANE_NOT_FOUND)*/
                {
                    mesh.clear();
                }
            }
        }

        ros::spinOnce();

        loop_rate.sleep();

        if (i == 2)
        {
            ros::shutdown();
        }
    }

    image.free();
    mesh.clear();

    zed.disablePositionalTracking();
    zed.close();
    //coincedence(one, two);

    return 0;
}

void coincedence(Mesh one, Mesh two)
{
    std::vector<int> asd1 = one.getBoundaries();
    std::vector<int> asd2 = two.getBoundaries();

    cout << "# of one chunks: " << one.chunks.size() << endl;
    cout << "# of two chunks: " << two.chunks.size() << endl;
    cout << "# triangules of one " << one.getNumberOfTriangles() << endl;
    cout << "# triangules of two " << two.getNumberOfTriangles() << endl;

    cout << "size: " << asd1.size() << " One boundary: ";
    for (int i = 0; i < asd1.size(); i++)
    {
        cout << asd1.at(i) << " ";
    }
    cout << endl;

    cout << "size: " << asd2.size() << " Two boundary: ";
    for (int i = 0; i < asd2.size(); i++)
    {
        cout << asd2.at(i) << " ";
    }

    cout << endl;
    uint nTriangles = 0;

    size_t ptIdx = 0;
    for (size_t t = 0; t < one.getNumberOfTriangles(); t++)
    {
        cout << "t value: " << t << endl;
        for (int p = 0; p < 3; p++)
        {
            uint vIdx = one.triangles[t][p];
            cout << "x: " << one.vertices[vIdx][0] << " y: " << one.vertices[vIdx][1] << " z: " << one.vertices[vIdx][2] << endl;
            uint old = nTriangles;
            findSimilarTriangle(nTriangles, one.vertices[vIdx], two);
            if (old != nTriangles)
                cout << "old: " << old << " new: " << nTriangles << endl;
        }
        cout << endl;
    }
    cout << "Found coincidence of " << nTriangles << " triangles!" << endl;
}
void findSimilarTriangle(uint &n, sl::float3 vertices, Mesh haystack)
{

    std::vector<float> tolerance = {0.1, 0.1, 0.1};
    size_t ptIdx = 0;
    for (size_t t = 0; t < haystack.getNumberOfTriangles(); t++)
    {
        for (int p = 0; p < 3; p++)
        {
            uint vIdx = haystack.triangles[t][p];
            // tolerance in x:  x2 - tol < x1 < x2+tol
            if (haystack.vertices[vIdx][0] + tolerance.at(0) > vertices[0] && haystack.vertices[vIdx][0] - tolerance.at(0) < vertices[0])
            {

                if (haystack.vertices[vIdx][1] + tolerance.at(1) > vertices[1] && haystack.vertices[vIdx][1] - tolerance.at(1) < vertices[1])
                {
                    if (haystack.vertices[vIdx][2] + tolerance.at(2) > vertices[2] && haystack.vertices[vIdx][2] - tolerance.at(2) < vertices[2])
                    {
                        n++;
                        /*
                        cout << "t value: " << t << endl;
                        cout << "target x: " << vertices[0] << " y: " << vertices[1] << " z: " << vertices[2] << endl;
                        cout << "x: " << haystack.vertices[vIdx][0] << " y: " << haystack.vertices[vIdx][1] << " z: " << haystack.vertices[vIdx][2] << endl;
                        n++;
                        cout << endl;
                        */
                    }
                }
            }
        }
    }

    // cout << "Found coincidence of " << cnt << " triangles!" << endl;
}

void parseArgs(int argc, char **argv, sl::InitParameters &param)
{
    if (argc > 1 && string(argv[1]).find(".svo") != string::npos)
    {
        // SVO input mode
        param.input.setFromSVOFile(argv[1]);
        cout << "[Sample] Using SVO File input: " << argv[1] << endl;
    }
    else if (argc > 1 && string(argv[1]).find(".svo") == string::npos)
    {
        string arg = string(argv[1]);
        unsigned int a, b, c, d, port;
        if (sscanf(arg.c_str(), "%u.%u.%u.%u:%d", &a, &b, &c, &d, &port) == 5)
        {
            // Stream input mode - IP + port
            string ip_adress = to_string(a) + "." + to_string(b) + "." + to_string(c) + "." + to_string(d);
            param.input.setFromStream(sl::String(ip_adress.c_str()), port);
            cout << "[Sample] Using Stream input, IP : " << ip_adress << ", port : " << port << endl;
        }
        else if (sscanf(arg.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4)
        {
            // Stream input mode - IP only
            param.input.setFromStream(sl::String(argv[1]));
            cout << "[Sample] Using Stream input, IP : " << argv[1] << endl;
        }
        else if (arg.find("HD2K") != string::npos)
        {
            param.camera_resolution = sl::RESOLUTION::HD2K;
            cout << "[Sample] Using Camera in resolution HD2K" << endl;
        }
        else if (arg.find("HD1080") != string::npos)
        {
            param.camera_resolution = sl::RESOLUTION::HD1080;
            cout << "[Sample] Using Camera in resolution HD1080" << endl;
        }
        else if (arg.find("HD720") != string::npos)
        {
            param.camera_resolution = sl::RESOLUTION::HD720;
            cout << "[Sample] Using Camera in resolution HD720" << endl;
        }
        else if (arg.find("VGA") != string::npos)
        {
            param.camera_resolution = sl::RESOLUTION::VGA;
            cout << "[Sample] Using Camera in resolution VGA" << endl;
        }
    }
    else
    {
        // Default
    }
}

void meshToPlaneMarker(visualization_msgs::MarkerPtr &plane_marker, sl::Mesh mesh, sl::Pose pose)
{

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    static int plane_mesh_id = 0;
    ros::Time ts = ros::Time::now();
    plane_marker->header.stamp = ts;
    // Set the marker action.  Options are ADD and DELETE
    plane_marker->action = visualization_msgs::Marker::ADD;
    plane_marker->lifetime = ros::Duration(0.1);

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