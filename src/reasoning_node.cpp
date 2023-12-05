#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <sstream>
#include <iostream>
#include <fstream>

#include <rosprolog/rosprolog_client/PrologClient.h>

#include <world_percept_assig3/UpdateObjectList.h>

using namespace std;

class Reasoner
{
private: 
    PrologClient pl_;
    int ID_;

    std::string srv_assert_knowledge_name_;
    ros::ServiceServer assert_knowledge_srv_;                            // Advertise service to assert knowledge in the ontology

    //Variable to save our preference to save or not the asserted queries
    bool m_query_flag_save;
public:

    Reasoner(ros::NodeHandle &nh)
    {
        ROS_INFO_STREAM("Wait for the Prolog service...");

        if(pl_.waitForServer())
            pl_ = PrologClient("/rosprolog", true);

        ID_=0; //Global variable to include in the asserted instances

        srv_assert_knowledge_name_ = "assert_knowledge";
        assert_knowledge_srv_ = nh.advertiseService(srv_assert_knowledge_name_, &Reasoner::srv_assert_callback, this);

        this->m_query_flag_save=false;
    };

    ~Reasoner(){

    };

   //TODO A03.T02: This function should open a file if the path of the file is founc, otherwise it should print out that the file or directory were not found (0.4 pts)
    void setOutQueriesFile(string QueryfileName)
{
    ofstream queryFile;
    queryFile.open(QueryfileName, ios::out | ios::app); // Open for writing and append to it if it exists

    if (!queryFile) // Check if the file was successfully opened
    {
        ROS_ERROR_STREAM("File or directory not found: " << QueryfileName);
        return;
    }

    ROS_INFO_STREAM("File opened successfully: " << QueryfileName);

        this->m_query_flag_save=true; //This means that I want to save the queries in a file
    }

private:    


     /**
     * @brief Callback function for the service that adds objects to the map_objects list
     *
     * @param Request requested object to be added to the knowledge base
     * @param Respose response from the service when the object has been asserted (true/false)
     */
    bool srv_assert_callback(world_percept_assig2::UpdateObjectList::Request &req,
                             world_percept_assig2::UpdateObjectList::Response &res)
    {
        ROS_INFO_STREAM("Got new object: " << req.object_name);
        std::string object = req.object_name;

        if(getClass(object))
        {
            res.confirmation = assertKnowledge(object);
        }
        else
        {
            ROS_WARN_STREAM("The object has not been asserted in the knowledge base");
            res.confirmation = false;
        }

        return res.confirmation;
    }


    bool getClass(const std::string &className)
    {
        std::string query = "get_class('" + className + "')";
        ROS_INFO_STREAM("Prolog query: " << query);

        PrologQuery bdgs = pl_.query(query);

        for (const auto &it : bdgs) 
        {
            ROS_INFO_STREAM("A new class was created in the ontology: " << className);
            return true;
        }

        ROS_ERROR_STREAM("The class was not created in the ontology: " << className); 
        return false;
    }

    bool assertKnowledge(const std::string &objectName)
    {
        std::string instanceName = objectName + "_" + std::to_string(ID_++);
        std::string query = "create_instance_from_class('" + objectName + "', '" + instanceName + "', Instance)";

        ROS_INFO_STREAM("Prolog query: " << query);

        PrologQuery bdgs = pl_.query(query);

        bool success = false;
        for(PrologQuery::iterator it = bdgs.begin(); it != bdgs.end(); it++)
        {
            success = true;
            ROS_INFO_STREAM("New instance asserted in knowledge base: " << instanceName);
            break;
        }

        bdgs.finish();

        if (!success)
        {
            ROS_ERROR_STREAM("Instance could not be asserted in knowledge base: " << instanceName);
        }

        return success;
    }

}; //class Reasoner

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "reasoning_node");

  ros::NodeHandle nh;   
  
  Reasoner myReasoner(nh);

   //+ Information about the path for the file that will save the queries
  std::string saveFilePath;
  saveFilePath = argv[1]; // This means that the node expects a path value as input. This means that we need to run this node as follows: rosrun world_percept

 
   bool saveQueries_flag; //variable that will receive the value from the yaml file
   nh.getParam("read_prolog_queries/save_flag", saveQueries_flag); // Retrieve the variable from the yaml file

   //TODO A03.T02: Retrieve the variable from the yaml file and save it in the new variable "saveQueries_flag" (0.2 pts) 
   std::string savedQueryFile;

   if(saveQueries_flag)
    { //If the flag is true, then I will configure the file to save the asserted queries
         //TODO A03.T02: Include the code to load the rosparam from a yaml file (0.4 pts)
         // First define a new string variable "savedQueryFile"
         // Then load the yaml parameter in the new variable

        std::string yamlFilePath;
        nh.getParam("read_prolog_queries/saved_query_file", yamlFilePath);
        //This node now needs a path as input when we run it. This path is addedd to the obtain variable from the yaml file, as follows
        savedQueryFile= saveFilePath+savedQueryFile;
        ROS_INFO_STREAM("query_file: "<< savedQueryFile);

        //Now we call a new function which will create and open the new file
        myReasoner.setOutQueriesFile(savedQueryFile);
     
    }

  ros::spin();

  
  return 0;
}