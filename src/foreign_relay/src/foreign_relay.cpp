#include <foreign_relay.h>

void foreign_advertise(const std::string &type)
{
    XmlRpc::XmlRpcClient *client = g_xmlrpc_manager->getXMLRPCClient(g_host, g_port, "/");
    XmlRpc::XmlRpcValue args, result;
    args[0] = ros::this_node::getName();
    args[1] = g_foreign_topic;
    args[2] = type;
    args[3] = g_xmlrpc_manager->getServerURI();
    if (!client->execute("registerPublisher", args, result))
    {
        ROS_FATAL("Failed to contact foreign master at [%s:%d] to register [%s].", g_host.c_str(), g_port, g_foreign_topic.c_str());
        g_error = true;
        ros::shutdown();
    }
    ros::XMLRPCManager::instance()->releaseXMLRPCClient(client);
}

void foreign_unadvertise()
{
    XmlRpc::XmlRpcClient *client = g_xmlrpc_manager->getXMLRPCClient(g_host, g_port, "/");
    XmlRpc::XmlRpcValue args, result;
    args[0] = ros::this_node::getName();
    args[1] = g_foreign_topic;
    args[2] = g_xmlrpc_manager->getServerURI();
    if (!client->execute("unregisterPublisher", args, result))
    {
        ROS_ERROR("Failed to contact foreign master at [%s:%d] to unregister [%s].", g_host.c_str(), g_port, g_foreign_topic.c_str());
        g_error = true;
    }
    ros::XMLRPCManager::instance()->releaseXMLRPCClient(client);
}

void foreign_subscribe()
{
    XmlRpc::XmlRpcClient *client =
            g_xmlrpc_manager->getXMLRPCClient(g_host, g_port, "/");
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();
    args[1] = g_foreign_topic;
    args[2] = "*";
    args[3] = g_xmlrpc_manager->getServerURI();
    if (!client->execute("registerSubscriber", args, result))
    {
        ROS_FATAL("Failed to contact foreign master at [%s:%d] to register [%s].",
                  g_host.c_str(), g_port, g_foreign_topic.c_str());
        g_error = true;
        ros::shutdown();
        return;
    }

    {
        // Horrible hack: the response from registerSubscriber() can contain a
        // list of current publishers.  But we don't have a way of injecting them
        // into roscpp here.  Now, if we get a publisherUpdate() from the master,
        // everything will work.  So, we ask the master if anyone is currently
        // publishing the topic, grab the advertised type, use it to advertise
        // ourselves, then unadvertise, triggering a publisherUpdate() along the
        // way.
        XmlRpc::XmlRpcValue args, result, payload;
        args[0] = ros::this_node::getName();
        args[1] = std::string("");
        if(!client->execute("getPublishedTopics", args, result))
        {
            ROS_FATAL("Failed to call getPublishedTopics() on foreign master at [%s:%d]",
                      g_host.c_str(), g_port);
            g_error = true;
            ros::shutdown();
            return;
        }
        if (!ros::XMLRPCManager::instance()->validateXmlrpcResponse("getPublishedTopics", result, payload))
        {
            ROS_FATAL("Failed to get validate response to getPublishedTopics() from foreign master at [%s:%d]",
                      g_host.c_str(), g_port);
            g_error = true;
            ros::shutdown();
            return;
        }
        for(int i=0;i<payload.size();i++)
        {
            std::string topic = std::string(payload[i][0]);
            std::string type = std::string(payload[i][1]);

            if(topic == g_foreign_topic)
            {
                foreign_advertise(type);
                foreign_unadvertise();
                break;
            }
        }
    }

    ros::XMLRPCManager::instance()->releaseXMLRPCClient(client);
}

void foreign_unsubscribe()
{
    XmlRpc::XmlRpcClient *client = g_xmlrpc_manager->getXMLRPCClient(g_host, g_port, "/");
    XmlRpc::XmlRpcValue args, result;
    args[0] = ros::this_node::getName();
    args[1] = g_foreign_topic;
    args[2] = g_xmlrpc_manager->getServerURI();
    if (!client->execute("unregisterSubscriber", args, result))
    {
        ROS_ERROR("Failed to contact foreign master at [%s:%d] to unregister [%s].", g_host.c_str(), g_port, g_foreign_topic.c_str());
        g_error = true;
    }
    ros::XMLRPCManager::instance()->releaseXMLRPCClient(client);
}

void in_cb(const topic_tools::ShapeShifter::ConstPtr& msg)
{
    if (!g_advertised)
    {
        if(g_mode == MODE_SUB)
        {
            // Advertise locally
            g_pub = msg->advertise(*g_node, g_local_topic, 10);
            ROS_INFO("Advertised locally as %s, with type %s",
                     g_local_topic.c_str(),msg->getDataType().c_str());
        }
        else
        {
            // We advertise locally as a hack, to get things set up properly.
            g_pub = msg->advertise(*g_node, g_foreign_topic, 10);
            // Advertise at the foreign master.
            foreign_advertise(msg->getDataType());
            ROS_INFO("Advertised foreign as %s, with type %s",
                     g_foreign_topic.c_str(), msg->getDataType().c_str());
        }
        g_advertised = true;
    }
    g_pub.publish(msg);
}

int main(int argc, char **argv)
{
    if (argc < 5)
    {
        ROS_FATAL(USAGE);
        return 1;
    }
    if(std::string(argv[1]) == "adv")
        g_mode = MODE_ADV;
    else if(std::string(argv[1]) == "sub")
        g_mode = MODE_SUB;
    else
    {
        ROS_FATAL(USAGE);
        return 1;
    }
    std::string foreign_master_uri;
    foreign_master_uri = argv[2];
    g_foreign_topic = argv[3];
    g_local_topic = argv[4];
    std::string local_topic_basename;
    if(!getBaseName(g_local_topic, local_topic_basename))
    {
        ROS_FATAL("Failed to extract basename from topic [%s]",
                  g_local_topic.c_str());
        return 1;
    }
    if (!ros::network::splitURI(foreign_master_uri, g_host, g_port))
    {
        ROS_FATAL("Couldn't parse the foreign master URI [%s] into a host:port pair.", foreign_master_uri.c_str());
        return 1;
    }

    char buf[1024];
    snprintf(buf, sizeof(buf), "%d", ros::master::getPort());
    ros::init(argc, argv, local_topic_basename + string("_foreign_relay_") + buf + "_" + ((g_mode == MODE_ADV) ? "adv" : "sub"),
              ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::NodeHandle pnh("~");
    g_node = &pnh;

    ros::Subscriber sub;
    if(g_mode == MODE_SUB)
    {
        // We subscribe locally as a hack, to get our callback set up properly.
        sub = n.subscribe<ShapeShifter>(g_foreign_topic, 10, &in_cb,
                                        ros::TransportHints().unreliable());
        // Subscribe at foreign master.
        foreign_subscribe();
    }
    else
    {
        // Subscribe at local master.
        sub = n.subscribe<ShapeShifter>(g_local_topic, 10, &in_cb);
    }

    ros::spin();

    if(g_mode == MODE_SUB)
        foreign_unsubscribe();
    else
        foreign_unadvertise();

    return g_error;
}

