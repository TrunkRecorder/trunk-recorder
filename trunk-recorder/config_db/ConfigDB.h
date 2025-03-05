#ifndef CONFIG_DB_H
#define CONFIG_DB_H

#include <string>
#include <vector>
#include <json.hpp>

using json = nlohmann::json;


class ConfigDB {
public:
    /**
    * @brief Read the config from a MongoDB collection and return it as a JSON array.
    * 
    * @param uri_str MongoDB connection URI (e.g., "mongodb://user:password@localhost:27017/mydatabase").
    * @param db_name Name of the database.
    * @param collection_name Name of the collection.
    * @param config_name Name of the configuration to retrieve.
    * @return nlohmann::json A JSON array representing the collection's documents. Returns an empty array if there's an error.
    */
    json readConfigToJson(const std::string& uri_str, const std::string& db_name, const std::string& collection_name, const std::string& config_name);

    /**
     * @brief Default constructor.
     */
    ConfigDB();

    /**
     * @brief Destructor.
     */
    ~ConfigDB();
};

#endif // CONFIG_DB_H
