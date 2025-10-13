using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using MongoDB.Driver;
using SimTMDG.Database.Entity;

namespace SimTMDG.Database
{
    public class MongoService
    {
        private static readonly MongoClient _client;
        private readonly IMongoDatabase _database;
        private readonly IMongoCollection<AtcsResult> _atcsResultCollection;

        static MongoService()
        {
            string connectionString = "mongodb://atcs:HgsVxomvAXvUW3z0@nosql.smartsystem.id:27017/atcs";
            MongoUrl mongoUrl = MongoUrl.Create(connectionString);
            _client = new MongoClient(mongoUrl);
        }

        public MongoService()
        {
            string databaseName = "atcs";
            _database = _client.GetDatabase(databaseName);
            _atcsResultCollection = _database.GetCollection<AtcsResult>("atcs-results");
        }


        public List<string> TestConnection()
        {
            List<string> result = new List<string>();

            try
            {
                result = _database.ListCollectionNames().ToList();
                return result;
            } catch(Exception e)
            {
                Console.WriteLine(e.ToString());
                return result;
            }
        }

        public async Task<List<AtcsResult>>GetAsync() =>
            await _atcsResultCollection.Find(_ => true).ToListAsync();

        public async Task<AtcsResult>GetAsync(string id) =>
            await _atcsResultCollection.Find(result => result.Guid == id).FirstOrDefaultAsync();
    }

}
