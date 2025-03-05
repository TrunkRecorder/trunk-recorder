#include "ConfigDB.h"
#include <iostream>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/json.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/exception/exception.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/shared_ptr.hpp>

using json = nlohmann::json;

namespace logging = boost::log;
namespace keywords = boost::log::keywords;
namespace sinks = boost::log::sinks;

ConfigDB::ConfigDB() {}

ConfigDB::~ConfigDB() {}

json ConfigDB::readConfigToJson(const std::string& uri_str, const std::string& db_name, const std::string& collection_name, const std::string& config_name) {
    json result_json = json::array(); // Initialize as an empty JSON array

    try {
        // Create a MongoDB client
        BOOST_LOG_TRIVIAL(info) << "Connecting to MongoDB";
        mongocxx::instance instance{}; // This should be done once in the program, but for simplicity it's here.
        mongocxx::uri uri(uri_str);
        mongocxx::client client(uri);

        // Get the database and collection
        mongocxx::database db = client[db_name];
        mongocxx::collection collection = db[collection_name];

         // Filter for the specified config_name
         bsoncxx::builder::stream::document filter_builder;
         filter_builder << "name" << config_name;

        // Find the document in the collection
        auto maybe_result = collection.find_one(filter_builder.view());

         if (maybe_result) {
            auto doc = maybe_result->view();
            // Get the value of the "config" element and then convert it to JSON
            std::string doc_json_str;
            doc_json_str = bsoncxx::to_json(doc);
            BOOST_LOG_TRIVIAL(info) << "Document found in collection: " << doc_json_str;
            result_json = json::parse(doc_json_str);
            BOOST_LOG_TRIVIAL(info) << "Document Parsed Successfully";
         } else {
            BOOST_LOG_TRIVIAL(warning) << "Document not found in collection: " << collection_name << " with name: " << config_name;
            return json::object();
         }
    } catch (const mongocxx::exception& e) {
        BOOST_LOG_TRIVIAL(error) << "Error reading from MongoDB: " << e.what();
        // Handle the exception as needed (e.g., return an empty JSON or rethrow)
        return json::object();
    } catch (const std::exception& e) {
        BOOST_LOG_TRIVIAL(error) << "General error: " << e.what();
        return json::object();
    }

    return result_json;
}
