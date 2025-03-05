/*
* This script reads a JSON file and imports the data to a MongoDB collection.*
* Usage: node mongo-import.js <json-file-path> <mongodb-connection-string>*
* Example: node mongo-import.js data.json mongodb+srv://<username>:<password>@<cluster>/<database>*
*/

const fs = require('fs');
const { MongoClient } = require('mongodb');

// Check if required arguments were provided
if (process.argv.length < 4) {
console.log('Usage: node mongo-import.js <json-file-path> <mongodb-connection-string>');
process.exit(1);
}

// Get command line arguments
const jsonFilePath = process.argv[2];
const mongoUri = process.argv[3];

// Function to read JSON file and import to MongoDB
async function importJsonToMongo() {
let client;

try {
    // Read and parse the JSON file
    const jsonData = JSON.parse(fs.readFileSync(jsonFilePath, 'utf8'));
    console.log(`Successfully read JSON file: ${jsonFilePath}`);
    
    // Connect to MongoDB
    client = new MongoClient(mongoUri);
    await client.connect();
    console.log('Connected to MongoDB Atlas');
    
    // Get reference to the database and collection
    const database = client.db('trgui');
    const collection = database.collection('configs');
    
    // Insert the data
    const result = Array.isArray(jsonData) 
    ? await collection.insertMany(jsonData)
    : await collection.insertOne(jsonData);
    
    console.log('Data successfully uploaded to MongoDB Atlas');
    console.log('Items inserted:', Array.isArray(jsonData) ? result.insertedCount : 1);
    
} catch (error) {
    console.error('Error:', error.message);
    process.exit(1);
} finally {
    // Close the connection
    if (client) {
    await client.close();
    console.log('MongoDB connection closed');
    }
}
}

// Run the import function
importJsonToMongo().catch(console.error);

