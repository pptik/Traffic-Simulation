using System;
using MongoDB.Bson;
using MongoDB.Bson.Serialization.Attributes;
using SimTMDG.Road;

namespace SimTMDG.Database.Entity
{
    public class TrafficLightRecomentadion
    {
        [BsonElement("green_sec"), BsonRepresentation(BsonType.Int32)]
        public int greenSec { get; set; }

        [BsonElement("yellow_sec"), BsonRepresentation(BsonType.Int32)]
        public int yellowSec { get; set; }

        [BsonElement("red_sec"), BsonRepresentation(BsonType.Int32)]
        public int redSec { get; set; }
    }

    public class AtcsResult
    {
        [BsonId]
        [BsonElement("_id"), BsonRepresentation(BsonType.ObjectId)]
        public string Id { get; set; }

        [BsonElement("guid_proses"), BsonRepresentation(BsonType.String)]
        public string GuidProcess { get; set; }

        [BsonElement("camera_id"), BsonRepresentation(BsonType.String)]
        public string CameraId { get; set; }

        [BsonElement("processed_at"), BsonRepresentation(BsonType.Double)]
        public double ProcessedAt {  get; set; }

        [BsonElement("created_at"), BsonDateTimeOptions(Kind = DateTimeKind.Utc)]
        public DateTime CreatedAt { get; set; }

        [BsonElement("updated_at"), BsonDateTimeOptions(Kind = DateTimeKind.Utc)]
        public DateTime UpdatedAt { get; set; }

        [BsonElement("filename"), BsonRepresentation(BsonType.String)]
        public string FileName { get; set; }

        [BsonElement("filename_result"), BsonRepresentation(BsonType.String)]
        public string FileNameResult { get; set; }

        [BsonElement("average_speed_kmh"), BsonRepresentation(BsonType.Double)]
        public double AverageSpeedKmh { get; set; }

        [BsonElement("car"), BsonRepresentation(BsonType.Int32)]
        public int Car {  get; set; }

        [BsonElement("motorcycle"), BsonRepresentation(BsonType.Int32)]
        public int MotorCycle { get; set; }

        [BsonElement("bus"), BsonRepresentation(BsonType.Int32)]
        public int Bus {  get; set; }

        [BsonElement("truck"), BsonRepresentation(BsonType.Int32)]
        public int Truck { get; set; }

        [BsonElement("total_vehicle"), BsonRepresentation(BsonType.Int32)]
        public int TotalVehicles { get; set; }

        [BsonElement("max_queue_length"), BsonRepresentation(BsonType.Int32)]
        public int MaxQueueLength { get; set; }

        [BsonElement("average_queue_length_m"), BsonRepresentation(BsonType.Double)]
        public double AverageMaxQueueLength { get; set; }

        [BsonElement("max_queue_length_m"), BsonRepresentation(BsonType.Double)]
        public double MaxQueueLengthM { get; set; }

        [BsonElement("queue_count"), BsonRepresentation(BsonType.Int32)]
        public int QueueCount { get; set; }

        [BsonElement("queue_length_m"), BsonRepresentation(BsonType.Double)]
        public double QueueLengthM { get; set; }

        [BsonElement("traffic_light"), BsonRepresentation(BsonType.String)]
        public ETrafficLight TrafficLight { get; set; }

        [BsonElement("recommendation")]
        public TrafficLightRecomentadion Recomentadion { get; set; }
    }
}
