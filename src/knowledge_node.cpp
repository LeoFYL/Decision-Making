#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <sstream>
#include <iostream>
#include <fstream>

#include <rosprolog/rosprolog_client/PrologClient.h>

#include <world_percept_assig3/loadKnowledge.h>

using namespace std;

class Knowledge_Loader
{
private: 
    PrologClient prolog_client_;
    int ID_;

    std::string srv_load_knowledge_name_;
    ros::ServiceServer load_knowledge_srv_;                            // Advertise service to assert knowledge in the ontology

    //Variable to save our preference to save or not the asserted queries
    bool m_query_flag_save;
    std::fstream SavedOpenFile;
public:

    Knowledge_Loader(ros::NodeHandle &nh)
    {
        ROS_INFO_STREAM("Wait for the Prolog service...");

        if(prolog_client_.waitForServer())
            prolog_client_ = PrologClient("/rosprolog", true);

        ID_=0; //Global variable to include in the asserted instances

       // srv_assert_knowledge_name_ = "load_knowledge";
       // assert_knowledge_srv_ = nh.advertiseService(srv_assert_knowledge_name_, &Reasoner2::callback_load_knowledge, this);
        srv_load_knowledge_name_ = "load_knowledge";
        load_knowledge_srv_ = nh.advertiseService(srv_load_knowledge_name_, &Knowledge_Loader::srv_load_callback, this);

        this->m_query_flag_save=false;
    };

    ~Knowledge_Loader(){

    };

    // void setQueryFile(string QueryfileName)
    // {
    //     SavedOpenFile.open(QueryfileName);
    //     if (!SavedOpenFile.is_open()) {                 
    //         cout <<"File not found" << endl;
    //     }
    //     this->m_query_flag_save=true; //This means that I want to save the queries in a file
    // }

void setQueryFile(std::string fileName_Q) {
    SavedOpenFile.open(fileName_Q); // Directly use the parameter

    if (!SavedOpenFile.is_open()) {
        std::cout << "File not found" << std::endl;
    } else {
        m_query_flag_save = true;
    }
}



private:    
    bool srv_load_callback(world_percept_assig3::loadKnowledge::Request &req,
                                 world_percept_assig3::loadKnowledge::Response &res)
    { 
        
        loadQueries();
        res.confirm = true;
        return true;
    }
    void loadQueries()
    {
        std::string line;
        while (std::getline(SavedOpenFile, line)) {
            ROS_INFO_STREAM("query: "<< line);
            PrologQuery bdgs = prolog_client_.query(line);
        }
    }
     /**
     * @brief Callback function for the service that adds objects to the map_objects list
     *
     * @param Request requested object to be added to t   he knowledge base
     * @param Respose response from the service when the object has been asserted (true/false)
     */
   

}; 

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "knowledge_node");

  ros::NodeHandle nh;   
  
  Knowledge_Loader myKnowledge_Loader(nh);

   //+ Information about the path for the file that will save the queries
  std::string saveFilePath;
  saveFilePath = argv[1]; // This means that the node expects a path value as input. This means that we need to run this node as follows: rosrun world_percept

 
   bool saveQueries_flag; //variable that will receive the value from the yaml file

  
   std::string savedQueryFile;
   nh.getParam("read_prolog_queries/save_flag", saveQueries_flag);
    

   if(saveQueries_flag)
    { //If the flag is true, then I will configure the file to save the asserted queries
         //TODO A03.T02: Include the code to load the rosparam from a yaml file (0.4 pts)
         // First define a new string variable "savedQueryFile"
         // Then load the yaml parameter in the new variable

        nh.getParam("read_prolog_queries/saved_query_file", savedQueryFile);
        //This node now needs a path as input when we run it. This path is addedd to the obtain variable from the yaml file, as follows
        savedQueryFile= saveFilePath+savedQueryFile;
        ROS_INFO_STREAM("query_file: "<< savedQueryFile);

        //Now we call a new function which will create and open the new file
        myKnowledge_Loader.setQueryFile(savedQueryFile);
     
    }

  ros::spin();

  
  return 0;
}