// main.cpp - Complete GTA-style game with improved systems (SFML 3.0 compatible)
#include <SFML/Graphics.hpp>
#include <iostream>
#include <random>
#include <ctime>
#include <vector>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <memory>
#include <string>
#include <deque>

// Game constants - BALANCED
const int MAP_SIZE = 100;
const int TILE_SIZE = 64;
const int BLOCK_SIZE = 8;
const float BASE_CAR_SPEED = 80.0f; // Ridotto da 120.0f
const float VIEW_DISTANCE = 1000.0f; // Ridotto per performance
const int MAX_CARS = 60; // Ridotto da 80
const int MAX_PEDESTRIANS = 80; // Aumentato per più vita in città
const float LANE_WIDTH = 48.0f;
const float SIDEWALK_WIDTH = 16.0f;

// Global textures for different vehicle types
static sf::Texture streetTexture;
static sf::Texture sidewalkTexture;
static sf::Texture buildingTexture;
static sf::Texture parkTexture;
static sf::Texture parkingTexture;
static sf::Texture PlayerTextureIdle;
static sf::Texture PlayerTextureWalk;
static sf::Texture PlayerTextureShoot;
static sf::Texture CarTexture1;
static sf::Texture sedanTexture;
static sf::Texture suvTexture;
static sf::Texture sportsCarTexture;
static sf::Texture truckTexture;
static sf::Texture compactTexture;
static sf::Texture busTexture;

// Pedestrian textures
static sf::Texture pedestrianCitizenTexture;
static sf::Texture pedestrianBusinessmanTexture;
static sf::Texture pedestrianElderlyTexture;
static sf::Texture pedestrianChildTexture;
static sf::Texture pedestrianJoggerTexture;
static sf::Texture pedestrianWomanTexture;
static sf::Texture pedestrianPoliceTexture;

static bool hasSedanTexture = false;
static bool hasSuvTexture = false;
static bool hasSportsCarTexture = false;
static bool hasTruckTexture = false;
static bool hasCompactTexture = false;
static bool hasBusTexture = false;

// Pedestrian texture flags
static bool hasPedestrianCitizen = false;
static bool hasPedestrianBusinessman = false;
static bool hasPedestrianElderly = false;
static bool hasPedestrianChild = false;
static bool hasPedestrianJogger = false;
static bool hasPedestrianWoman = false;
static bool hasPedestrianPolice = false;

// Texture loading flags
static bool hasStreetTexture = false;
static bool hasSidewalkTexture = false;
static bool hasBuildingTexture = false;
static bool hasParkTexture = false;
static bool hasParkingTexture = false;
static bool hasPlayerIdle = false;
static bool hasPlayerWalk = false;
static bool hasPlayerShoot = false;
static bool hasCarTexture1 = false;

// Vehicle types enum
enum VehicleType
{
    SEDAN = 0,
    SUV = 1,
    SPORTS_CAR = 2,
    TRUCK = 3,
    COMPACT = 4,
    BUS = 5
};

// Pedestrian enums
enum PedestrianState {
    WALKING = 0,
    WAITING = 1,
    RUNNING = 2,  // Quando spaventato dalle auto
    CROSSING = 3, // Attraversamento strada
    IDLE = 4      // Fermo in un punto
};

enum PedestrianType {
    CITIZEN = 0,
    BUSINESSMAN = 1,
    ELDERLY = 2,
    CHILD = 3,
    JOGGER = 4,
    WOMAN = 5,
    POLICE = 6
};

enum TextureResolution {
    LOW_RES = 32,    // 32px per dispositivi mobili
    MED_RES = 64,    // 64px standard
    HIGH_RES = 128   // 128px per HD
};

// Map cell types
enum MapType
{
    NOTHING = 0,
    STREET = 1,
    BUILDING = 2,
    PARK = 3,
    PARKING = 4,
    SIDEWALK = 5,
};

// Direction enum (coordinated with SFML rotation)
enum Direction
{
    NORTH = 0, // Up
    EAST = 1,  // Right
    SOUTH = 2, // Down
    WEST = 3,  // Left
    NONE = -1
};

// Vehicle dimensions structure
struct VehicleDimensions {
    sf::Vector2f baseSize;      // Dimensione base in pixels
    sf::Vector2f collisionBox; // Hitbox per collisioni (più piccola)
    sf::Vector2f textureSize;   // Dimensione texture originale
    float textureScale;         // Scala per adattare texture
    
    VehicleDimensions() = default;
    VehicleDimensions(sf::Vector2f base, sf::Vector2f collision, sf::Vector2f texture) 
        : baseSize(base), collisionBox(collision), textureSize(texture) {
        // Calcola scala automaticamente
        textureScale = std::min(base.x / texture.x, base.y / texture.y);
    }
};

// Pedestrian structures
struct PedestrianWaypoint {
    sf::Vector2f position;
    float waitTime;
    bool isIntersection;
    bool isCrossingPoint;
    
    PedestrianWaypoint(sf::Vector2f pos, float wait = 0.f, bool intersection = false, bool crossing = false)
        : position(pos), waitTime(wait), isIntersection(intersection), isCrossingPoint(crossing) {}
};

struct PedestrianProperties {
    PedestrianType type;
    float speed;
    float panicThreshold; // Distanza dalle auto per iniziare a correre
    sf::Color color;
    sf::Vector2f size;
    float waitTimeMultiplier;
    sf::Texture* texture;
    
    PedestrianProperties(PedestrianType t, float s, float panic, sf::Color c, sf::Vector2f sz, float waitMult, sf::Texture* tex)
        : type(t), speed(s), panicThreshold(panic), color(c), size(sz), waitTimeMultiplier(waitMult), texture(tex) {}
};

// Vehicle properties structure
struct VehicleProperties
{
    VehicleType type;
    sf::Vector2f size;
    sf::Texture *texture;
    float speedMultiplier;
    float aggressiveness;
    float turnRate;

    VehicleProperties() = default;
    VehicleProperties(VehicleType t, sf::Vector2f s, sf::Texture *tex, float speed, float aggr, float turn)
        : type(t), size(s), texture(tex), speedMultiplier(speed), aggressiveness(aggr), turnRate(turn) {}
};

// Forward declarations
struct MapGenerator;
struct Car;

// Pedestrian Database
class PedestrianDatabase {
public:
    static PedestrianProperties getRandomPedestrianProperties(std::mt19937 &rng) {
        std::uniform_int_distribution<int> typeDist(0, 6);
        PedestrianType type = static_cast<PedestrianType>(typeDist(rng));
        
        switch (type) {
            case CITIZEN:
                return PedestrianProperties(
                    CITIZEN, 45.f, 80.f, 
                    sf::Color(100, 150, 200), sf::Vector2f(16.f, 16.f), 1.0f,
                    hasPedestrianCitizen ? &pedestrianCitizenTexture : nullptr
                );
                
            case BUSINESSMAN:
                return PedestrianProperties(
                    BUSINESSMAN, 55.f, 70.f,
                    sf::Color(50, 50, 50), sf::Vector2f(16.f, 16.f), 0.7f,
                    hasPedestrianBusinessman ? &pedestrianBusinessmanTexture : nullptr
                );
                
            case ELDERLY:
                return PedestrianProperties(
                    ELDERLY, 25.f, 100.f,
                    sf::Color(150, 150, 150), sf::Vector2f(14.f, 14.f), 2.0f,
                    hasPedestrianElderly ? &pedestrianElderlyTexture : nullptr
                );
                
            case CHILD:
                return PedestrianProperties(
                    CHILD, 35.f, 60.f,
                    sf::Color(255, 200, 100), sf::Vector2f(12.f, 12.f), 1.5f,
                    hasPedestrianChild ? &pedestrianChildTexture : nullptr
                );
                
            case JOGGER:
                return PedestrianProperties(
                    JOGGER, 90.f, 50.f,
                    sf::Color(255, 100, 100), sf::Vector2f(16.f, 16.f), 0.2f,
                    hasPedestrianJogger ? &pedestrianJoggerTexture : nullptr
                );
                
            case WOMAN:
                return PedestrianProperties(
                    WOMAN, 50.f, 75.f,
                    sf::Color(200, 150, 200), sf::Vector2f(16.f, 16.f), 1.1f,
                    hasPedestrianWoman ? &pedestrianWomanTexture : nullptr
                );
                
            case POLICE:
                return PedestrianProperties(
                    POLICE, 60.f, 30.f, // Police don't panic easily
                    sf::Color(0, 0, 200), sf::Vector2f(16.f, 16.f), 0.5f,
                    hasPedestrianPolice ? &pedestrianPoliceTexture : nullptr
                );
                
            default:
                return PedestrianProperties(
                    CITIZEN, 45.f, 80.f,
                    sf::Color(100, 150, 200), sf::Vector2f(16.f, 16.f), 1.0f,
                    hasPedestrianCitizen ? &pedestrianCitizenTexture : nullptr
                );
        }
    }
};

// Vehicle properties database
class VehicleDatabase {
private:
    static std::unordered_map<VehicleType, VehicleDimensions> dimensions;
    
public:
    static void initializeDimensions() {
        // Dimensioni basate su proporzioni reali di veicoli
        // Sedan: veicolo medio-piccolo
        dimensions[SEDAN] = VehicleDimensions(
            sf::Vector2f(40.f, 20.f),     // Base size (fits in lane)
            sf::Vector2f(36.f, 18.f),     // Collision box (90% of base)
            sf::Vector2f(64.f, 32.f)      // Expected texture size
        );
        
        // SUV: più largo e lungo
        dimensions[SUV] = VehicleDimensions(
            sf::Vector2f(44.f, 24.f),
            sf::Vector2f(40.f, 22.f),
            sf::Vector2f(64.f, 32.f)
        );
        
        // Sports Car: basso e stretto
        dimensions[SPORTS_CAR] = VehicleDimensions(
            sf::Vector2f(38.f, 18.f),
            sf::Vector2f(34.f, 16.f),
            sf::Vector2f(64.f, 32.f)
        );
        
        // Truck: molto grande
        dimensions[TRUCK] = VehicleDimensions(
            sf::Vector2f(48.f, 28.f),
            sf::Vector2f(44.f, 26.f),
            sf::Vector2f(96.f, 48.f)      // Texture più grande
        );
        
        // Compact: piccolo
        dimensions[COMPACT] = VehicleDimensions(
            sf::Vector2f(32.f, 18.f),
            sf::Vector2f(28.f, 16.f),
            sf::Vector2f(48.f, 32.f)
        );
        
        // Bus: molto lungo
        dimensions[BUS] = VehicleDimensions(
            sf::Vector2f(56.f, 24.f),
            sf::Vector2f(52.f, 22.f),
            sf::Vector2f(128.f, 48.f)     // Texture molto grande
        );
    }
    
    static VehicleDimensions getDimensions(VehicleType type) {
        auto it = dimensions.find(type);
        if (it != dimensions.end()) {
            return it->second;
        }
        return dimensions[SEDAN]; // Default fallback
    }
    
    static VehicleProperties getRandomVehicleProperties(std::mt19937 &rng) {
        std::uniform_int_distribution<int> typeDist(0, 5);
        VehicleType type = static_cast<VehicleType>(typeDist(rng));
        
        VehicleDimensions dims = getDimensions(type);
        
        // Proprietà aggiornate con dimensioni ottimizzate
        switch (type) {
            case SEDAN:
                return VehicleProperties(
                    SEDAN, dims.baseSize, 
                    (hasSedanTexture ? &sedanTexture : nullptr),
                    1.0f, 0.3f, 0.8f
                );
                
            case SUV:
                return VehicleProperties(
                    SUV, dims.baseSize,
                    (hasSuvTexture ? &suvTexture : nullptr),
                    0.85f, 0.2f, 0.6f
                );
                
            case SPORTS_CAR:
                return VehicleProperties(
                    SPORTS_CAR, dims.baseSize,
                    (hasSportsCarTexture ? &sportsCarTexture : nullptr),
                    1.4f, 0.8f, 1.2f
                );
                
            case TRUCK:
                return VehicleProperties(
                    TRUCK, dims.baseSize,
                    (hasTruckTexture ? &truckTexture : nullptr),
                    0.7f, 0.1f, 0.4f
                );
                
            case COMPACT:
                return VehicleProperties(
                    COMPACT, dims.baseSize,
                    (hasCompactTexture ? &compactTexture : nullptr),
                    1.1f, 0.4f, 1.0f
                );
                
            case BUS:
                return VehicleProperties(
                    BUS, dims.baseSize,
                    (hasBusTexture ? &busTexture : nullptr),
                    0.6f, 0.0f, 0.3f
                );
                
            default:
                return VehicleProperties(
                    SEDAN, dims.baseSize,
                    (hasSedanTexture ? &sedanTexture : nullptr),
                    1.0f, 0.3f, 0.8f
                );
        }
    }
};

// Initialize static member
std::unordered_map<VehicleType, VehicleDimensions> VehicleDatabase::dimensions;

// Utility functions
sf::Vector2f getOptimalTextureScale(const sf::Texture* texture, sf::Vector2f targetSize) {
    if (!texture) return sf::Vector2f(1.f, 1.f);
    
    sf::Vector2u textureSize = texture->getSize();
    return sf::Vector2f(
        targetSize.x / static_cast<float>(textureSize.x),
        targetSize.y / static_cast<float>(textureSize.y)
    );
}

// Road point structure for navigation
struct RoadPoint
{
    int gridX, gridY;
    float worldX, worldY;
    bool isIntersection;
    bool canGoNorth, canGoSouth, canGoEast, canGoWest;

    RoadPoint(int x, int y, bool intersection = false)
        : gridX(x), gridY(y), isIntersection(intersection)
    {
        worldX = x * TILE_SIZE + TILE_SIZE / 2.0f;
        worldY = y * TILE_SIZE + TILE_SIZE / 2.0f;
        canGoNorth = canGoSouth = canGoEast = canGoWest = false;
    }
};

// Enhanced road network class
class RoadNetwork
{
private:
    std::vector<std::unique_ptr<RoadPoint>> intersections;
    std::vector<std::unique_ptr<RoadPoint>> roadPoints;
    std::mt19937 rng;

public:
    RoadNetwork() : rng(std::random_device{}()) {}

    void generateNetwork(MapGenerator &gen);
    RoadPoint *getNearestRoadPoint(float x, float y);
    RoadPoint *getNearestIntersection(float x, float y);
    RoadPoint *getNextRoadPoint(int currentX, int currentY, Direction dir);
    Direction getValidDirections(RoadPoint *point, Direction currentDir);
    bool isValidDirection(RoadPoint *point, Direction dir);
    void drawNetwork(sf::RenderWindow &window);

    const std::vector<std::unique_ptr<RoadPoint>> &getIntersections() const { return intersections; }
    const std::vector<std::unique_ptr<RoadPoint>> &getRoadPoints() const { return roadPoints; }
};

// Enhanced map generator
struct MapGenerator
{
    int map[MAP_SIZE][MAP_SIZE];
    std::mt19937 rng;

    MapGenerator() : rng(static_cast<unsigned int>(time(nullptr)))
    {
        for (int i = 0; i < MAP_SIZE; i++)
            for (int j = 0; j < MAP_SIZE; j++)
                map[i][j] = NOTHING;
    }

    void generateGridCity()
    {
        // Generate perfect grid city with 8x8 blocks
        for (int i = 0; i < MAP_SIZE; i++)
        {
            for (int j = 0; j < MAP_SIZE; j++)
            {
                int posInBlockX = i % BLOCK_SIZE;
                int posInBlockY = j % BLOCK_SIZE;

                // Roads exactly at block centers
                bool isHorizontalRoad = (posInBlockY == 3);
                bool isVerticalRoad = (posInBlockX == 3);

                if (isHorizontalRoad || isVerticalRoad)
                {
                    map[i][j] = STREET;
                }
                else if (posInBlockY == 2 || posInBlockY == 4 ||
                         posInBlockX == 2 || posInBlockX == 4)
                {
                    // Sidewalks next to roads
                    map[i][j] = SIDEWALK;
                }
                else
                {
                    // Fill with buildings/parks
                    std::uniform_int_distribution<int> roll(1, 100);
                    int chance = roll(rng);
                    if (chance <= 70)
                        map[i][j] = BUILDING;
                    else if (chance <= 85)
                        map[i][j] = PARK;
                    else
                        map[i][j] = PARKING;
                }
            }
        }
    }

    int getCell(int x, int y) const
    {
        if (x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE)
            return map[x][y];
        return -1;
    }

    bool isRoadCenter(int x, int y) const
    {
        if (x < 0 || x >= MAP_SIZE || y < 0 || y >= MAP_SIZE)
            return false;

        // Must be road AND at exact block center
        if (map[x][y] != STREET)
            return false;

        int posInBlockX = x % BLOCK_SIZE;
        int posInBlockY = y % BLOCK_SIZE;

        return (posInBlockX == 3 || posInBlockY == 3);
    }

    bool isIntersection(int x, int y) const
    {
        if (!isRoadCenter(x, y))
            return false;

        int posInBlockX = x % BLOCK_SIZE;
        int posInBlockY = y % BLOCK_SIZE;

        // Intersection = where horizontal AND vertical roads meet
        return (posInBlockX == 3 && posInBlockY == 3);
    }

    bool isHorizontalRoad(int x, int y) const
    {
        if (!isRoadCenter(x, y))
            return false;
        return (y % BLOCK_SIZE == 3);
    }

    bool isVerticalRoad(int x, int y) const
    {
        if (!isRoadCenter(x, y))
            return false;
        return (x % BLOCK_SIZE == 3);
    }
};

// Enhanced road network implementation
void RoadNetwork::generateNetwork(MapGenerator &gen)
{
    intersections.clear();
    roadPoints.clear();

    // Find all road points
    for (int i = 0; i < MAP_SIZE; i++)
    {
        for (int j = 0; j < MAP_SIZE; j++)
        {
            if (gen.isRoadCenter(i, j))
            {
                if (gen.isIntersection(i, j))
                {
                    auto inter = std::make_unique<RoadPoint>(i, j, true);

                    // Check possible directions
                    inter->canGoNorth = gen.isRoadCenter(i, j - 1);
                    inter->canGoSouth = gen.isRoadCenter(i, j + 1);
                    inter->canGoEast = gen.isRoadCenter(i + 1, j);
                    inter->canGoWest = gen.isRoadCenter(i - 1, j);

                    intersections.push_back(std::move(inter));
                }
                else
                {
                    // Normal road point
                    auto road = std::make_unique<RoadPoint>(i, j, false);
                    roadPoints.push_back(std::move(road));
                }
            }
        }
    }

    std::cout << "Generated " << intersections.size() << " intersections and "
              << roadPoints.size() << " road points\n";
}

RoadPoint *RoadNetwork::getNearestRoadPoint(float x, float y)
{
    RoadPoint *nearest = nullptr;
    float minDist = std::numeric_limits<float>::max();

    // Check intersections first
    for (auto &inter : intersections)
    {
        float dx = x - inter->worldX;
        float dy = y - inter->worldY;
        float dist = std::sqrt(dx * dx + dy * dy);

        if (dist < minDist)
        {
            minDist = dist;
            nearest = inter.get();
        }
    }

    // Then normal road points
    for (auto &road : roadPoints)
    {
        float dx = x - road->worldX;
        float dy = y - road->worldY;
        float dist = std::sqrt(dx * dx + dy * dy);

        if (dist < minDist)
        {
            minDist = dist;
            nearest = road.get();
        }
    }

    return nearest;
}

RoadPoint *RoadNetwork::getNearestIntersection(float x, float y)
{
    RoadPoint *nearest = nullptr;
    float minDist = std::numeric_limits<float>::max();

    for (auto &inter : intersections)
    {
        float dx = x - inter->worldX;
        float dy = y - inter->worldY;
        float dist = std::sqrt(dx * dx + dy * dy);

        if (dist < minDist)
        {
            minDist = dist;
            nearest = inter.get();
        }
    }

    return nearest;
}

RoadPoint *RoadNetwork::getNextRoadPoint(int currentX, int currentY, Direction dir)
{
    int nextX = currentX;
    int nextY = currentY;

    switch (dir)
    {
    case NORTH:
        nextY--;
        break;
    case SOUTH:
        nextY++;
        break;
    case EAST:
        nextX++;
        break;
    case WEST:
        nextX--;
        break;
    default:
        return nullptr;
    }

    // Search in intersections
    for (auto &inter : intersections)
    {
        if (inter->gridX == nextX && inter->gridY == nextY)
            return inter.get();
    }

    // Search in road points
    for (auto &road : roadPoints)
    {
        if (road->gridX == nextX && road->gridY == nextY)
            return road.get();
    }

    return nullptr;
}

Direction RoadNetwork::getValidDirections(RoadPoint *point, Direction currentDir)
{
    if (!point || !point->isIntersection)
        return currentDir; // If not intersection, continue straight

    std::vector<Direction> validDirs;
    Direction opposite = static_cast<Direction>((currentDir + 2) % 4);

    // Add all valid directions except opposite
    if (point->canGoNorth && currentDir != opposite)
        validDirs.push_back(NORTH);
    if (point->canGoEast && currentDir != opposite)
        validDirs.push_back(EAST);
    if (point->canGoSouth && currentDir != opposite)
        validDirs.push_back(SOUTH);
    if (point->canGoWest && currentDir != opposite)
        validDirs.push_back(WEST);

    if (validDirs.empty())
        return opposite; // Forced to turn back

    // 70% chance to continue straight if possible
    if (std::find(validDirs.begin(), validDirs.end(), currentDir) != validDirs.end())
    {
        if (std::uniform_real_distribution<float>(0, 1)(rng) < 0.7f)
            return currentDir;
    }

    // Choose random direction
    std::uniform_int_distribution<size_t> dirDist(0, validDirs.size() - 1);
    return validDirs[dirDist(rng)];
}

bool RoadNetwork::isValidDirection(RoadPoint *point, Direction dir)
{
    if (!point)
        return false;

    switch (dir)
    {
    case NORTH:
        return point->canGoNorth;
    case SOUTH:
        return point->canGoSouth;
    case EAST:
        return point->canGoEast;
    case WEST:
        return point->canGoWest;
    default:
        return false;
    }
}

void RoadNetwork::drawNetwork(sf::RenderWindow &window)
{
    // Draw intersections in yellow
    for (auto &inter : intersections)
    {
        sf::CircleShape circle(4.f);
        circle.setFillColor(sf::Color::Yellow);
        circle.setOrigin(sf::Vector2f(4.f, 4.f));
        circle.setPosition(sf::Vector2f(inter->worldX, inter->worldY));
        window.draw(circle);
    }

    // Draw road points in green
    for (auto &road : roadPoints)
    {
        sf::CircleShape circle(2.f);
        circle.setFillColor(sf::Color::Green);
        circle.setOrigin(sf::Vector2f(2.f, 2.f));
        circle.setPosition(sf::Vector2f(road->worldX, road->worldY));
        window.draw(circle);
    }
}

// Pedestrian class - DECLARATION ONLY
class Pedestrian {
private:
    sf::RectangleShape shape; // Cambiato da CircleShape a RectangleShape per texture
    sf::Vector2f position;
    sf::Vector2f velocity;
    PedestrianProperties properties;
    PedestrianState currentState;
    
    // AI e pathfinding migliorato
    std::deque<PedestrianWaypoint> waypointQueue;
    PedestrianWaypoint* currentTarget;
    float stateTimer;
    float avoidanceRadius;
    
    // Comportamenti
    bool isScared;
    sf::Vector2f fleeDirection;
    float fearTimer;
    
    // Movimento sui marciapiedi
    sf::Vector2f lastSafeSidewalkPosition;
    bool isOnSidewalk;
    
    std::mt19937* rng;
    
public:
    float distanceToPlayer;
    bool isActive;
    
    Pedestrian(sf::Vector2f startPos, std::mt19937* randomGen);
    
    void generateRandomPath();
    sf::Vector2f generateSidewalkPosition();
    sf::Vector2f generateCrossingPoint();
    void setNextTarget();
    void update(float deltaTime, sf::Vector2f playerPos, const std::vector<std::unique_ptr<Car>>& cars, MapGenerator& gen);
    void checkCarAvoidance(const std::vector<std::unique_ptr<Car>>& cars);
    void updateWalking(float deltaTime, MapGenerator& gen);
    void updateWaiting(float deltaTime);
    void updateRunning(float deltaTime, MapGenerator& gen);
    void updateCrossing(float deltaTime, const std::vector<std::unique_ptr<Car>>& cars);
    void updateIdle(float deltaTime);
    void draw(sf::RenderWindow& window);
    
    // Utility functions for sidewalk movement
    bool isPositionOnSidewalk(sf::Vector2f pos, MapGenerator& gen);
    sf::Vector2f getNearestSidewalkPosition(sf::Vector2f pos, MapGenerator& gen);
    sf::Vector2f generateRandomSidewalkWaypoint();
    
    sf::Vector2f getPosition() const { return position; }
    PedestrianType getType() const { return properties.type; }
    PedestrianState getState() const { return currentState; }
};

// Car structure with vehicle variety and damage system
struct Car
{
    sf::RectangleShape carShape;
    float x, y;
    float baseSpeed;
    float currentSpeed;
    Direction currentDirection;
    VehicleProperties properties;

    // Enhanced AI state
    RoadPoint *currentRoadPoint;
    RoadPoint *targetRoadPoint;
    float stopTimer;
    float distanceToPlayer;
    bool isActive;
    bool isStopping;

    // Car damage system
    float health = 1000.0f;
    float maxHealth = 1000.0f;
    bool isDestroyed = false;
    float smokeTimer = 0.f;
    sf::Vector2f lastPosition;
    float lastSpeed = 0.f;
    bool isOnFire = false;
    float fireTimer = 0.f;
    float explosionTimer = -1.f; // -1 = not exploded, >=0 = exploding

    // Sistema collisioni migliorato
    float lastCollisionTime = 0.f;
    float collisionCooldown = 1.0f; // Cooldown di 1 secondo tra collisioni
    sf::Vector2f collisionVelocity = sf::Vector2f(0.f, 0.f);
    sf::Vector2f collisionBox; // Dimensioni collision box (più piccola del visual)

    // Custom behavior
    float aggressionTimer;
    float honkTimer;
    bool isHonking;

    // Grid-based movement
    bool isMovingToTarget;
    float targetX, targetY;

    // Random and network references
    std::mt19937 *rng;
    RoadNetwork *roadNetwork;
    MapGenerator *mapGen;

    // Sistema danni migliorato
    void updateCollisionDamage(float deltaTime, float currentSpeed, bool hasCollision, sf::Vector2f moveVector) {
        lastCollisionTime += deltaTime;
        
        if (hasCollision && lastCollisionTime >= collisionCooldown) {
            // Soglia minima per danni: 200 pixels/sec invece di 100
            float minDamageSpeed = 200.0f;
            
            if (currentSpeed > minDamageSpeed) {
                // Sistema di danni graduali basato sulla velocità
                float excessSpeed = currentSpeed - minDamageSpeed;
                float baseDamage = excessSpeed * 0.15f; // Ridotto da 0.3f
                
                // Moltiplicatore basato sul tipo di veicolo
                float vehicleMultiplier = 1.0f;
                switch (properties.type) {
                    case TRUCK:
                    case BUS:
                        vehicleMultiplier = 0.6f; // Veicoli pesanti sono più resistenti
                        break;
                    case SPORTS_CAR:
                        vehicleMultiplier = 1.4f; // Auto sportive più fragili
                        break;
                    case COMPACT:
                        vehicleMultiplier = 1.2f;
                        break;
                    default:
                        vehicleMultiplier = 1.0f;
                }
                
                float finalDamage = baseDamage * vehicleMultiplier;
                
                // Limita danni massimi per impatto singolo
                finalDamage = std::min(finalDamage, 100.0f);
                
                takeDamage(finalDamage);
                lastCollisionTime = 0.f; // Reset cooldown
                
                std::cout << "COLLISION! Speed: " << (int)currentSpeed 
                          << " | Damage: " << (int)finalDamage 
                          << " | Vehicle: " << getVehicleTypeName() << std::endl;
            }
        }
    }

    // Funzione collision check migliorata
    bool checkBuildingCollision(float newX, float newY, MapGenerator &gen) {
        sf::Vector2f halfBox = collisionBox / 2.0f;
        
        std::vector<sf::Vector2f> checkPoints = {
            {newX - halfBox.x, newY - halfBox.y},
            {newX + halfBox.x, newY - halfBox.y},
            {newX - halfBox.x, newY + halfBox.y},
            {newX + halfBox.x, newY + halfBox.y}
        };
        
        for (const auto& point : checkPoints) {
            int gridX = static_cast<int>(point.x / TILE_SIZE);
            int gridY = static_cast<int>(point.y / TILE_SIZE);
            
            if (gen.getCell(gridX, gridY) == BUILDING) {
                return true;
            }
        }
        
        return false;
    }

    // Damage system methods
    void takeDamage(float damage) {
        if (isDestroyed) return;
        
        health -= damage;
        if (health <= 0.f) {
            health = 0.f;
            if (!isDestroyed) {
                isDestroyed = true;
                explosionTimer = 2.0f; // 2 seconds of explosion
                std::cout << "Car destroyed!" << std::endl;
            }
        }
        
        // Car catches fire when health < 200
        if (health < 200.f && health > 0.f) {
            isOnFire = true;
        }
    }
    
    float getHealthPercentage() const {
        return health / maxHealth;
    }
    
    sf::Color getDamageColor() const {
        float healthPct = getHealthPercentage();
        
        if (healthPct > 0.8f) {
            return sf::Color::White; // Perfect
        } else if (healthPct > 0.6f) {
            return sf::Color(220, 220, 220); // Light scratches
        } else if (healthPct > 0.4f) {
            return sf::Color(180, 180, 180); // Dented
        } else if (healthPct > 0.2f) {
            return sf::Color(120, 120, 120); // Heavily damaged
        } else {
            return sf::Color(80, 80, 80); // Nearly destroyed
        }
    }

    // Costruttore migliorato
    Car(float startX, float startY, Direction startDir, std::mt19937 *randomGen,
        RoadNetwork *network, MapGenerator *gen)
        : x(startX), y(startY), currentDirection(startDir),
          properties(VehicleDatabase::getRandomVehicleProperties(*randomGen)),
          currentRoadPoint(nullptr), targetRoadPoint(nullptr), stopTimer(0.f),
          distanceToPlayer(0.f), isActive(true), isStopping(false),
          aggressionTimer(0.f), honkTimer(0.f), isHonking(false),
          isMovingToTarget(false), targetX(startX), targetY(startY),
          rng(randomGen), roadNetwork(network), mapGen(gen)
    {
        // Set speed based on vehicle type
        baseSpeed = BASE_CAR_SPEED * properties.speedMultiplier;
        currentSpeed = baseSpeed;

        // Configure shape based on properties
        carShape.setSize(properties.size);
        carShape.setOrigin(sf::Vector2f(properties.size.x / 2.0f, properties.size.y / 2.0f));
        carShape.setPosition(sf::Vector2f(x, y));
        carShape.setTexture(properties.texture); // Use texture if available

        // Imposta collision box (90% delle dimensioni visive)
        collisionBox = sf::Vector2f(properties.size.x * 0.9f, properties.size.y * 0.9f);

        // Set rotation based on direction
        updateRotation();

        // Find current road point and initial target
        snapToNearestRoad();
        findNextTarget();
        
        // Initialize damage system
        lastPosition = sf::Vector2f(x, y);
    }

    std::string getVehicleTypeName() const
    {
        switch (properties.type)
        {
        case SEDAN:
            return "Sedan";
        case SUV:
            return "SUV";
        case SPORTS_CAR:
            return "Sports Car";
        case TRUCK:
            return "Truck";
        case COMPACT:
            return "Compact";
        case BUS:
            return "Bus";
        default:
            return "Unknown";
        }
    }

    void update(float deltaTime, std::vector<Car *> &allCars, sf::Vector2f playerPos)
    {
        // Damage system update
        if (explosionTimer >= 0.f) {
            explosionTimer -= deltaTime;
            if (explosionTimer <= 0.f) {
                // Car completely destroyed, becomes inactive
                isActive = false;
                return;
            }
        }
        
        if (isOnFire) {
            fireTimer += deltaTime;
            // Car on fire gradually loses health
            if (fireTimer > 0.5f) { // Every 0.5 seconds
                fireTimer = 0.f;
                takeDamage(25.f);
            }
        }
        
        if (smokeTimer > 0.f) {
            smokeTimer -= deltaTime;
        }

        // Calculate distance to player
        distanceToPlayer = std::sqrt(std::pow(x - playerPos.x, 2) + std::pow(y - playerPos.y, 2));

        // Deactivate if too far
        if (distanceToPlayer > VIEW_DISTANCE)
        {
            isActive = false;
            return;
        }
        isActive = true;

        // Update timers
        if (aggressionTimer > 0.f)
            aggressionTimer -= deltaTime;
        if (honkTimer > 0.f)
            honkTimer -= deltaTime;

        // Intersection stop handling with aggression-based behavior
        if (stopTimer > 0.f)
        {
            stopTimer -= deltaTime;

            // Aggressive vehicles wait less time
            float aggressionReduction = properties.aggressiveness * 0.3f;
            if (aggressionReduction > 0.f && aggressionTimer <= 0.f)
            {
                stopTimer -= aggressionReduction;
                aggressionTimer = 2.0f; // Aggression cooldown
            }

            if (stopTimer <= 0.f)
            {
                // After stop, decide new direction
                if (currentRoadPoint && currentRoadPoint->isIntersection)
                {
                    currentDirection = getDirectionChoice(currentRoadPoint, currentDirection);
                    updateRotation();
                }
                findNextTarget();
            }
            return;
        }

        sf::Vector2f playerPosition = playerPos;
        if (distanceToPlayer < 100.0f)
        {
            // If player is very close, slow down a bit
            baseSpeed = (BASE_CAR_SPEED * properties.speedMultiplier) * 0.5f;
        }
        else
        {
            baseSpeed = BASE_CAR_SPEED * properties.speedMultiplier;
        }

        // Reduce speed based on damage
        if (isDestroyed) return;
        baseSpeed *= getHealthPercentage();

        // Check collisions with custom behavior
        checkCollisions(allCars);

        // Adjust speed based on situation and personality
        updateSpeed(deltaTime, allCars);

        // Movement towards target
        if (targetRoadPoint)
        {
            moveToTarget(deltaTime);

            // Check if we reached target
            float dx = x - targetRoadPoint->worldX;
            float dy = y - targetRoadPoint->worldY;
            float dist = std::sqrt(dx * dx + dy * dy);

            if (dist < 20.0f)
            {
                // Snap to exact target
                x = targetRoadPoint->worldX;
                y = targetRoadPoint->worldY;
                currentRoadPoint = targetRoadPoint;

                // If intersection, stop (time based on vehicle type)
                if (currentRoadPoint->isIntersection)
                {
                    float baseStopTime = (properties.type == BUS) ? 1.5f : 0.8f;
                    stopTimer = baseStopTime + std::uniform_real_distribution<float>(0.f, 0.5f)(*rng);
                }
                else
                {
                    // Continue to next point
                    findNextTarget();
                }
            }
        }

        // Update graphics
        carShape.setPosition(sf::Vector2f(x, y));

        // Calculate current speed for damage calculations
        sf::Vector2f currentPos(x, y);
        float deltaDistance = std::sqrt(std::pow(currentPos.x - lastPosition.x, 2) + 
                                       std::pow(currentPos.y - lastPosition.y, 2));
        lastSpeed = deltaDistance / deltaTime;
        lastPosition = currentPos;
    }

    Direction getDirectionChoice(RoadPoint *point, Direction currentDir)
    {
        if (!point || !point->isIntersection)
            return currentDir;

        std::vector<Direction> validDirs;
        Direction opposite = static_cast<Direction>((currentDir + 2) % 4);

        // Add all valid directions except opposite
        if (point->canGoNorth && currentDir != opposite)
            validDirs.push_back(NORTH);
        if (point->canGoEast && currentDir != opposite)
            validDirs.push_back(EAST);
        if (point->canGoSouth && currentDir != opposite)
            validDirs.push_back(SOUTH);
        if (point->canGoWest && currentDir != opposite)
            validDirs.push_back(WEST);

        if (validDirs.empty())
            return opposite;

        // Straight probability based on vehicle type
        float straightProbability = 0.7f;
        if (properties.type == SPORTS_CAR)
            straightProbability = 0.5f; // Sports cars more unpredictable
        if (properties.type == BUS || properties.type == TRUCK)
            straightProbability = 0.9f; // Heavy vehicles more predictable

        if (std::find(validDirs.begin(), validDirs.end(), currentDir) != validDirs.end())
        {
            if (std::uniform_real_distribution<float>(0.f, 1.f)(*rng) < straightProbability)
                return currentDir;
        }

        // Choose random direction
        std::uniform_int_distribution<size_t> dirDist(0, validDirs.size() - 1);
        return validDirs[dirDist(*rng)];
    }

    void updateSpeed(float deltaTime, std::vector<Car *> & /*allCars*/)
    {
        float targetSpeed = baseSpeed;

        if (isStopping)
        {
            // Brake rate based on vehicle type
            float brakeRate = (properties.type == TRUCK || properties.type == BUS) ? 200.0f : 400.0f;
            currentSpeed = std::max(0.0f, currentSpeed - brakeRate * deltaTime);

            // Aggressive vehicles honk when blocked
            if (properties.aggressiveness > 0.5f && currentSpeed < 20.0f && honkTimer <= 0.f)
            {
                honkTimer = 0.5f;
                isHonking = true;
            }
        }
        else
        {
            // Acceleration based on vehicle type
            float accelRate = 300.0f;
            if (properties.type == SPORTS_CAR)
                accelRate = 500.0f;
            if (properties.type == TRUCK || properties.type == BUS)
                accelRate = 150.0f;

            currentSpeed = std::min(targetSpeed, currentSpeed + accelRate * deltaTime);
        }
    }

    void moveToTarget(float deltaTime)
    {
        if (!targetRoadPoint)
            return;

        float dx = targetRoadPoint->worldX - x;
        float dy = targetRoadPoint->worldY - y;
        float distance = std::sqrt(dx * dx + dy * dy);

        if (distance < 1.0f)
            return;

        float moveDistance = currentSpeed * deltaTime;
        if (moveDistance > distance)
            moveDistance = distance;

        x += (dx / distance) * moveDistance;
        y += (dy / distance) * moveDistance;
    }

    void findNextTarget()
    {
        if (!roadNetwork || !currentRoadPoint)
            return;

        // Find next point in current direction
        targetRoadPoint = roadNetwork->getNextRoadPoint(
            currentRoadPoint->gridX,
            currentRoadPoint->gridY,
            currentDirection);

        // If no target found, try alternative directions
        if (!targetRoadPoint && currentRoadPoint->isIntersection)
        {
            Direction newDir = getDirectionChoice(currentRoadPoint, currentDirection);
            if (newDir != currentDirection)
            {
                currentDirection = newDir;
                updateRotation();
                targetRoadPoint = roadNetwork->getNextRoadPoint(
                    currentRoadPoint->gridX,
                    currentRoadPoint->gridY,
                    currentDirection);
            }
            else
            {
                // If choice is to remain, still try to find next
                targetRoadPoint = roadNetwork->getNextRoadPoint(
                    currentRoadPoint->gridX,
                    currentRoadPoint->gridY,
                    currentDirection);
            }
        }
    }

    void checkCollisions(std::vector<Car *> &allCars)
    {
        isStopping = false;
        float checkDistance = 70.0f;

        // Heavy vehicles have greater safety distance
        if (properties.type == TRUCK || properties.type == BUS)
            checkDistance = 90.0f;
        // Sports cars have smaller distance (more aggressive)
        else if (properties.type == SPORTS_CAR && properties.aggressiveness > 0.6f)
            checkDistance = 50.0f;

        for (Car *other : allCars)
        {
            if (other == this || !other->isActive || other->isDestroyed)
                continue;

            float dx = other->x - x;
            float dy = other->y - y;
            float dist = std::sqrt(dx * dx + dy * dy);

            // Car-to-car collision damage
            if (dist < 30.0f) { // Very close collision
                float relativeSpeed = std::abs(currentSpeed - other->currentSpeed);
                if (relativeSpeed > 50.f) { // Only if significant speed
                    float damage = relativeSpeed * 0.3f;
                    takeDamage(damage);
                    other->takeDamage(damage * 0.5f); // Other car takes less damage
                    
                    std::cout << "Car collision! Damage: " << (int)damage << std::endl;
                }
            }

            if (dist < checkDistance)
            {
                // Check if ahead in same direction
                bool isAhead = false;
                float laneWidth = (properties.type == BUS || properties.type == TRUCK) ? 40.0f : 30.0f;

                switch (currentDirection)
                {
                case NORTH:
                    isAhead = (dy < -10 && std::abs(dx) < laneWidth);
                    break;
                case SOUTH:
                    isAhead = (dy > 10 && std::abs(dx) < laneWidth);
                    break;
                case EAST:
                    isAhead = (dx > 10 && std::abs(dy) < laneWidth);
                    break;
                case WEST:
                    isAhead = (dx < -10 && std::abs(dy) < laneWidth);
                    break;
                default:
                    break;
                }

                if (isAhead)
                {
                    isStopping = true;
                    break;
                }
            }
        }
    }

    void snapToNearestRoad()
    {
        if (!roadNetwork)
            return;

        RoadPoint *nearest = roadNetwork->getNearestRoadPoint(x, y);
        if (nearest)
        {
            x = nearest->worldX;
            y = nearest->worldY;
            currentRoadPoint = nearest;
        }
    }

    void updateRotation()
    {
        float healthPct = getHealthPercentage();
        float rotationOffset = 0.f;
        
        // Very damaged car has unstable controls
        if (healthPct < 0.3f) {
            static float timeCounter = 0.f;
            timeCounter += 0.016f; // Approximate deltaTime
            rotationOffset = (std::sin(timeCounter * 10) * 5.f) * (1.f - healthPct);
        }

        switch (currentDirection)
        {
        case NORTH:
            carShape.setRotation(sf::degrees(270 + rotationOffset));
            break;
        case EAST:
            carShape.setRotation(sf::degrees(0 + rotationOffset));
            break;
        case SOUTH:
            carShape.setRotation(sf::degrees(90 + rotationOffset));
            break;
        case WEST:
            carShape.setRotation(sf::degrees(180 + rotationOffset));
            break;
        default:
            break;
        }
    }

    void draw(sf::RenderWindow &window)
    {
        if (!isActive) return;
        
        if (explosionTimer > 0.f) {
            // Explosion effect
            sf::CircleShape explosion(50.f + (2.f - explosionTimer) * 50.f);
            explosion.setOrigin(sf::Vector2f(explosion.getRadius(), explosion.getRadius()));
            explosion.setPosition(sf::Vector2f(x, y));
            
            // Changing explosion color
            float intensity = explosionTimer / 2.f;
            std::uint8_t red = 255;
            std::uint8_t green = static_cast<std::uint8_t>(255 * intensity);
            std::uint8_t blue = 0;
            explosion.setFillColor(sf::Color(red, green, blue, static_cast<std::uint8_t>(intensity * 255)));
            
            window.draw(explosion);
            
            // Still draw car but black/burned
            carShape.setFillColor(sf::Color(20, 20, 20));
            window.draw(carShape);
            return;
        }
        
        // Color based on damage
        carShape.setFillColor(getDamageColor());
        
        // Flashing effect for honking or when heavily damaged
        if ((isHonking && honkTimer > 0.f) || (getHealthPercentage() < 0.2f && health > 0.f)) {
            static float timeCounter = 0.f;
            timeCounter += 0.016f;
            float intensity = 0.8f + 0.2f * std::sin(timeCounter * 20.0f);
            sf::Color currentColor = getDamageColor();
            sf::Color flashColor = sf::Color(
                static_cast<std::uint8_t>(std::clamp(currentColor.r * intensity, 0.f, 255.f)),
                static_cast<std::uint8_t>(std::clamp(currentColor.g * intensity, 0.f, 255.f)),
                static_cast<std::uint8_t>(std::clamp(currentColor.b * intensity, 0.f, 255.f))
            );
            carShape.setFillColor(flashColor);
        }
        
        window.draw(carShape);
        
        // Particle effects
        float healthPct = getHealthPercentage();
        
        // Light smoke when damaged
        if (healthPct < 0.6f && healthPct > 0.2f) {
            if (smokeTimer <= 0.f) {
                smokeTimer = 0.1f; // Every 0.1 seconds
                // Draw gray smoke particle
                sf::CircleShape smoke(2.f);
                smoke.setPosition(sf::Vector2f(x + (rand() % 20 - 10), y - 10));
                smoke.setFillColor(sf::Color(100, 100, 100, 100));
                window.draw(smoke);
            }
        }
        
        // Heavy smoke when heavily damaged
        if (healthPct < 0.4f && health > 0.f) {
            for (int i = 0; i < 3; i++) {
                sf::CircleShape heavySmoke(3.f + i);
                heavySmoke.setPosition(sf::Vector2f(x + (rand() % 30 - 15), y - 15 - i*5));
                heavySmoke.setFillColor(sf::Color(60, 60, 60, 150 - i*30));
                window.draw(heavySmoke);
            }
        }
        
        // Flames when on fire
        if (isOnFire && health > 0.f) {
            for (int i = 0; i < 5; i++) {
                sf::CircleShape flame(1.f + rand() % 4);
                flame.setPosition(sf::Vector2f(x + (rand() % 40 - 20), y + (rand() % 20 - 10)));
                flame.setFillColor(sf::Color(255, rand() % 100 + 100, 0, 200));
                window.draw(flame);
            }
        }
    }
};

// NOW DEFINE PEDESTRIAN METHODS AFTER CAR IS FULLY DEFINED
Pedestrian::Pedestrian(sf::Vector2f startPos, std::mt19937* randomGen) 
    : position(startPos), properties(PedestrianDatabase::getRandomPedestrianProperties(*randomGen)),
      currentState(WALKING), currentTarget(nullptr), stateTimer(0.f),
      avoidanceRadius(50.f), isScared(false), fearTimer(0.f), rng(randomGen),
      distanceToPlayer(0.f), isActive(true), isOnSidewalk(true) {
    
    // Configura shape come sprite rettangolare per texture
    shape.setSize(properties.size);
    shape.setFillColor(properties.color);
    shape.setOrigin(sf::Vector2f(properties.size.x / 2.f, properties.size.y / 2.f));
    shape.setPosition(position);
    
    // Imposta texture se disponibile
    if (properties.texture) {
        shape.setTexture(properties.texture);
    }
    
    velocity = sf::Vector2f(0.f, 0.f);
    lastSafeSidewalkPosition = startPos;
    
    // Genera path iniziale
    generateRandomPath();
}

bool Pedestrian::isPositionOnSidewalk(sf::Vector2f pos, MapGenerator& gen) {
    int gridX = static_cast<int>(pos.x / TILE_SIZE);
    int gridY = static_cast<int>(pos.y / TILE_SIZE);
    
    int cellType = gen.getCell(gridX, gridY);
    return (cellType == SIDEWALK || cellType == PARK);
}

sf::Vector2f Pedestrian::getNearestSidewalkPosition(sf::Vector2f pos, MapGenerator& gen) {
    int centerX = static_cast<int>(pos.x / TILE_SIZE);
    int centerY = static_cast<int>(pos.y / TILE_SIZE);
    
    // Cerca in un raggio crescente
    for (int radius = 1; radius <= 3; radius++) {
        for (int dx = -radius; dx <= radius; dx++) {
            for (int dy = -radius; dy <= radius; dy++) {
                int checkX = centerX + dx;
                int checkY = centerY + dy;
                
                if (gen.getCell(checkX, checkY) == SIDEWALK) {
                    return sf::Vector2f(
                        checkX * TILE_SIZE + TILE_SIZE / 2.f,
                        checkY * TILE_SIZE + TILE_SIZE / 2.f
                    );
                }
            }
        }
    }
    
    return pos; // Fallback
}

sf::Vector2f Pedestrian::generateRandomSidewalkWaypoint() {
    // Genera waypoint casuali che siano garantiti sui marciapiedi
    int attempts = 0;
    sf::Vector2f waypoint;
    
    do {
        int blockX = (*rng)() % (MAP_SIZE / BLOCK_SIZE);
        int blockY = (*rng)() % (MAP_SIZE / BLOCK_SIZE);
        
        // Posizioni specifiche dei marciapiedi nel sistema a griglia
        int localX, localY;
        
        // Scegli un lato del blocco per il marciapiede
        int side = (*rng)() % 4;
        switch (side) {
            case 0: // Lato nord
                localX = 1 + (*rng)() % (BLOCK_SIZE - 2);
                localY = 2; // Marciapiede nord
                break;
            case 1: // Lato sud
                localX = 1 + (*rng)() % (BLOCK_SIZE - 2);
                localY = 4; // Marciapiede sud
                break;
            case 2: // Lato ovest
                localX = 2; // Marciapiede ovest
                localY = 1 + (*rng)() % (BLOCK_SIZE - 2);
                break;
            case 3: // Lato est
                localX = 4; // Marciapiede est
                localY = 1 + (*rng)() % (BLOCK_SIZE - 2);
                break;
        }
        
        // Evita la strada centrale
        if (localX == 3) localX = ((*rng)() % 2 == 0) ? 2 : 4;
        if (localY == 3) localY = ((*rng)() % 2 == 0) ? 2 : 4;
        
        waypoint.x = (blockX * BLOCK_SIZE + localX) * TILE_SIZE + TILE_SIZE / 2.f;
        waypoint.y = (blockY * BLOCK_SIZE + localY) * TILE_SIZE + TILE_SIZE / 2.f;
        
        attempts++;
    } while (attempts < 20);
    
    return waypoint;
}

void Pedestrian::generateRandomPath() {
    waypointQueue.clear();
    
    // Genera più waypoint per percorsi più lunghi
    int numWaypoints = 4 + (*rng)() % 4; // 4-7 waypoint
    
    for (int i = 0; i < numWaypoints; i++) {
        sf::Vector2f waypoint = generateRandomSidewalkWaypoint();
        float waitTime = (0.3f + static_cast<float>((*rng)() % 15) / 10.f) * properties.waitTimeMultiplier;
        
        waypointQueue.push_back(PedestrianWaypoint(waypoint, waitTime));
    }
    
    // Occasionalmente aggiungi attraversamento
    if ((*rng)() % 3 == 0) { // 33% probabilità
        sf::Vector2f crossingPoint = generateCrossingPoint();
        waypointQueue.push_back(PedestrianWaypoint(crossingPoint, 2.0f, true, true));
    }
    
    setNextTarget();
}

sf::Vector2f Pedestrian::generateSidewalkPosition() {
    return generateRandomSidewalkWaypoint(); // Usa la nuova funzione migliorata
}

sf::Vector2f Pedestrian::generateCrossingPoint() {
    // Genera punto di attraversamento alle intersezioni
    int blockX = (*rng)() % (MAP_SIZE / BLOCK_SIZE - 1);
    int blockY = (*rng)() % (MAP_SIZE / BLOCK_SIZE - 1);
    
    float worldX = (blockX * BLOCK_SIZE + 3) * TILE_SIZE + TILE_SIZE / 2.f;
    float worldY = (blockY * BLOCK_SIZE + 3) * TILE_SIZE + TILE_SIZE / 2.f;
    
    return sf::Vector2f(worldX, worldY);
}

void Pedestrian::setNextTarget() {
    if (!waypointQueue.empty()) {
        currentTarget = &waypointQueue.front();
    } else {
        currentTarget = nullptr;
        // Rigenera path quando finisce
        generateRandomPath();
    }
}

void Pedestrian::update(float deltaTime, sf::Vector2f playerPos, const std::vector<std::unique_ptr<Car>>& cars, MapGenerator& gen) {
    // Calcola distanza dal player
    distanceToPlayer = std::sqrt(std::pow(position.x - playerPos.x, 2) + 
                                std::pow(position.y - playerPos.y, 2));
    
    // Deattiva se troppo lontano
    if (distanceToPlayer > VIEW_DISTANCE * 1.5f) {
        isActive = false;
        return;
    }
    isActive = true;
    
    // Update timers
    stateTimer += deltaTime;
    if (fearTimer > 0.f) fearTimer -= deltaTime;
    
    // Controlla se è ancora sui marciapiedi
    isOnSidewalk = isPositionOnSidewalk(position, gen);
    if (isOnSidewalk) {
        lastSafeSidewalkPosition = position;
    }
    
    // Controllo paura dalle auto
    checkCarAvoidance(cars);
    
    // State machine
    switch (currentState) {
        case WALKING:
            updateWalking(deltaTime, gen);
            break;
        case WAITING:
            updateWaiting(deltaTime);
            break;
        case RUNNING:
            updateRunning(deltaTime, gen);
            break;
        case CROSSING:
            updateCrossing(deltaTime, cars);
            break;
        case IDLE:
            updateIdle(deltaTime);
            break;
    }
    
    // Applica movimento con controllo marciapiedi
    sf::Vector2f newPosition = position + velocity * deltaTime;
    
    // Se il movimento lo porta fuori dai marciapiedi (e non è in attraversamento)
    if (currentState != CROSSING && !isPositionOnSidewalk(newPosition, gen)) {
        // Trova la posizione più vicina sui marciapiedi
        newPosition = getNearestSidewalkPosition(newPosition, gen);
        // Riduce la velocità per evitare oscillazioni
        velocity *= 0.3f;
    }
    
    position = newPosition;
    shape.setPosition(position);
    
    // Evita uscire dai confini
    position.x = std::clamp(position.x, 20.f, MAP_SIZE * TILE_SIZE - 20.f);
    position.y = std::clamp(position.y, 20.f, MAP_SIZE * TILE_SIZE - 20.f);
}

void Pedestrian::checkCarAvoidance(const std::vector<std::unique_ptr<Car>>& cars) {
    isScared = false;
    
    for (const auto& car : cars) {
        if (!car->isActive || car->isDestroyed) continue;
        
        float distToCar = std::sqrt(std::pow(position.x - car->x, 2) + 
                                   std::pow(position.y - car->y, 2));
        
        if (distToCar < properties.panicThreshold) {
            isScared = true;
            fearTimer = 2.0f;
            
            // Calcola direzione di fuga
            sf::Vector2f toMe = position - sf::Vector2f(car->x, car->y);
            float length = std::sqrt(toMe.x * toMe.x + toMe.y * toMe.y);
            if (length > 0) {
                fleeDirection = toMe / length;
            }
            
            // Cambia stato se non già in fuga
            if (currentState == WALKING || currentState == WAITING) {
                currentState = RUNNING;
                stateTimer = 0.f;
            }
            break;
        }
    }
}

void Pedestrian::updateWalking(float deltaTime, MapGenerator& gen) {
    if (!currentTarget) {
        currentState = IDLE;
        stateTimer = 0.f;
        return;
    }
    
    // Movimento verso target
    sf::Vector2f toTarget = currentTarget->position - position;
    float distance = std::sqrt(toTarget.x * toTarget.x + toTarget.y * toTarget.y);
    
    if (distance < 15.f) { // Aumentato da 10.f per evitare jittering
        // Raggiunto waypoint
        if (currentTarget->waitTime > 0.f) {
            currentState = WAITING;
            stateTimer = 0.f;
            velocity = sf::Vector2f(0.f, 0.f);
        } else {
            // Vai al prossimo waypoint
            waypointQueue.pop_front();
            setNextTarget();
        }
    } else {
        // Muovi verso target
        sf::Vector2f direction = toTarget / distance;
        velocity = direction * properties.speed;
        
        // Se non è in attraversamento, assicurati che il target sia su marciapiede
        if (!currentTarget->isCrossingPoint && !isPositionOnSidewalk(currentTarget->position, gen)) {
            // Trova un nuovo target sui marciapiedi
            waypointQueue.pop_front();
            sf::Vector2f newTarget = generateRandomSidewalkWaypoint();
            waypointQueue.push_front(PedestrianWaypoint(newTarget, 0.5f));
            setNextTarget();
        }
    }
}

void Pedestrian::updateWaiting(float deltaTime) {
    velocity = sf::Vector2f(0.f, 0.f);
    
    if (stateTimer >= currentTarget->waitTime) {
        waypointQueue.pop_front();
        setNextTarget();
        currentState = WALKING;
        stateTimer = 0.f;
    }
}

void Pedestrian::updateRunning(float deltaTime, MapGenerator& gen) {
    if (fearTimer <= 0.f && !isScared) {
        // Smetti di correre, torna sui marciapiedi se necessario
        if (!isOnSidewalk) {
            position = lastSafeSidewalkPosition;
        }
        currentState = WALKING;
        stateTimer = 0.f;
        return;
    }
    
    // Corri nella direzione opposta alle auto, ma cerca di rimanere sui marciapiedi
    sf::Vector2f fleeVelocity = fleeDirection * (properties.speed * 2.0f);
    sf::Vector2f testPosition = position + fleeVelocity * deltaTime;
    
    // Se la fuga lo porta fuori dai marciapiedi, cerca una direzione alternativa
    if (!isPositionOnSidewalk(testPosition, gen)) {
        // Cerca direzioni alternative sui marciapiedi
        for (int i = 0; i < 8; i++) {
            float angle = i * 45.0f * M_PI / 180.0f;
            sf::Vector2f altDirection(std::cos(angle), std::sin(angle));
            sf::Vector2f altVelocity = altDirection * (properties.speed * 1.5f);
            sf::Vector2f altTestPos = position + altVelocity * deltaTime;
            
            if (isPositionOnSidewalk(altTestPos, gen)) {
                velocity = altVelocity;
                return;
            }
        }
        // Se nessuna direzione funziona, rimani fermo
        velocity = sf::Vector2f(0.f, 0.f);
    } else {
        velocity = fleeVelocity;
    }
}

void Pedestrian::updateCrossing(float deltaTime, const std::vector<std::unique_ptr<Car>>& cars) {
    // Attraversamento strada - controlla traffico
    bool trafficClear = true;
    
    for (const auto& car : cars) {
        if (!car->isActive || car->isDestroyed) continue;
        
        float distToCar = std::sqrt(std::pow(position.x - car->x, 2) + 
                                   std::pow(position.y - car->y, 2));
        
        if (distToCar < 120.f) { // Area di controllo traffico aumentata
            trafficClear = false;
            break;
        }
    }
    
    if (trafficClear || stateTimer > 8.0f) { // Max 8 secondi di attesa
        // Procedi con attraversamento
        currentState = WALKING;
        stateTimer = 0.f;
    } else {
        // Aspetta che il traffico si liberi
        velocity = sf::Vector2f(0.f, 0.f);
    }
}

void Pedestrian::updateIdle(float deltaTime) {
    velocity = sf::Vector2f(0.f, 0.f);
    
    if (stateTimer > 2.0f) { // Idle per 2 secondi
        generateRandomPath();
        currentState = WALKING;
        stateTimer = 0.f;
    }
}

void Pedestrian::draw(sf::RenderWindow& window) {
    if (!isActive) return;
    
    // Cambia colore se spaventato
    if (isScared || fearTimer > 0.f) {
        shape.setFillColor(sf::Color::Red);
    } else {
        shape.setFillColor(properties.color);
    }
    
    window.draw(shape);
}

// Pedestrian Manager
class PedestrianManager {
private:
    std::vector<std::unique_ptr<Pedestrian>> pedestrians;
    std::mt19937 rng;
    float spawnTimer;
    
public:
    PedestrianManager() : rng(std::random_device{}()), spawnTimer(0.f) {}
    
    void initialize(MapGenerator& gen) {
        // Spawn iniziale di molti più pedoni per città viva
        for (int i = 0; i < 25; i++) {
            spawnRandomPedestrian(gen);
        }
        std::cout << "PedestrianManager initialized with " << pedestrians.size() << " pedestrians\n";
    }
    
    void spawnRandomPedestrian(MapGenerator& gen) {
        // Genera posizione casuale garantita sui marciapiedi
        int attempts = 0;
        sf::Vector2f spawnPos;
        
        do {
            int blockX = rng() % (MAP_SIZE / BLOCK_SIZE);
            int blockY = rng() % (MAP_SIZE / BLOCK_SIZE);
            
            // Scegli specificamente posizioni sui marciapiedi
            int side = rng() % 4;
            int localX, localY;
            
            switch (side) {
                case 0: // Lato nord del blocco
                    localX = 1 + rng() % (BLOCK_SIZE - 2);
                    localY = 2; // Marciapiede nord
                    break;
                case 1: // Lato sud del blocco
                    localX = 1 + rng() % (BLOCK_SIZE - 2);
                    localY = 4; // Marciapiede sud
                    break;
                case 2: // Lato ovest del blocco
                    localX = 2; // Marciapiede ovest
                    localY = 1 + rng() % (BLOCK_SIZE - 2);
                    break;
                case 3: // Lato est del blocco
                    localX = 4; // Marciapiede est
                    localY = 1 + rng() % (BLOCK_SIZE - 2);
                    break;
            }
            
            // Evita sovrapposizioni con le strade
            if (localX == 3) localX = (rng() % 2 == 0) ? 2 : 4;
            if (localY == 3) localY = (rng() % 2 == 0) ? 2 : 4;
            
            spawnPos.x = (blockX * BLOCK_SIZE + localX) * TILE_SIZE + TILE_SIZE / 2.f;
            spawnPos.y = (blockY * BLOCK_SIZE + localY) * TILE_SIZE + TILE_SIZE / 2.f;
            
            // Verifica che sia effettivamente sui marciapiedi
            int gridX = static_cast<int>(spawnPos.x / TILE_SIZE);
            int gridY = static_cast<int>(spawnPos.y / TILE_SIZE);
            if (gen.getCell(gridX, gridY) == SIDEWALK) {
                break; // Posizione valida trovata
            }
            
            attempts++;
        } while (attempts < 100);
        
        if (attempts < 100 && !isTooCloseToOthers(spawnPos, 25.f)) {
            auto newPedestrian = std::make_unique<Pedestrian>(spawnPos, &rng);
            pedestrians.push_back(std::move(newPedestrian));
        }
    }
    
    bool isTooCloseToOthers(sf::Vector2f position, float minDistance) {
        for (const auto& ped : pedestrians) {
            sf::Vector2f pedPos = ped->getPosition();
            float dist = std::sqrt(std::pow(position.x - pedPos.x, 2) + 
                                  std::pow(position.y - pedPos.y, 2));
            if (dist < minDistance) {
                return true;
            }
        }
        return false;
    }
    
    void update(float deltaTime, sf::Vector2f playerPos, const std::vector<std::unique_ptr<Car>>& cars, MapGenerator& gen) {
        // Update esistenti
        for (auto& ped : pedestrians) {
            ped->update(deltaTime, playerPos, cars, gen);
        }
        
        // Spawning più aggressivo per mantenere la città viva
        spawnTimer += deltaTime;
        if (spawnTimer > 1.5f) { // Ogni 1.5 secondi
            spawnTimer = 0.f;
            
            int activePedestrians = 0;
            for (const auto& ped : pedestrians) {
                if (ped->isActive) activePedestrians++;
            }
            
            if (activePedestrians < MAX_PEDESTRIANS) {
                // Spawn multipli per riempire velocemente
                int spawnCount = std::min(2, MAX_PEDESTRIANS - activePedestrians);
                for (int i = 0; i < spawnCount; i++) {
                    spawnRandomPedestrian(gen);
                }
            }
        }
        
        // Rimuovi pedoni troppo lontani o rimasti bloccati
        pedestrians.erase(
            std::remove_if(pedestrians.begin(), pedestrians.end(),
                [playerPos](const std::unique_ptr<Pedestrian>& ped) {
                    float dist = std::sqrt(std::pow(ped->getPosition().x - playerPos.x, 2) + 
                                          std::pow(ped->getPosition().y - playerPos.y, 2));
                    return dist > VIEW_DISTANCE * 2.5f; // Aumentato il range
                }),
            pedestrians.end()
        );
    }
    
    void draw(sf::RenderWindow& window) {
        for (auto& ped : pedestrians) {
            ped->draw(window);
        }
    }
    
    size_t getActivePedestrianCount() const {
        size_t count = 0;
        for (const auto& ped : pedestrians) {
            if (ped->isActive) count++;
        }
        return count;
    }
    
    size_t getTotalPedestrianCount() const {
        return pedestrians.size();
    }
};

// Enhanced Car Manager
class CarManager
{
private:
    std::vector<std::unique_ptr<Car>> cars;
    std::mt19937 rng;
    RoadNetwork roadNetwork;
    float spawnTimer;

public:
    CarManager() : rng(std::random_device{}()), spawnTimer(0.f) {}

    void initialize(MapGenerator &gen)
    {
        roadNetwork.generateNetwork(gen);

        // Initial spawn - fill the map
        for (int i = 0; i < 15; i++)
        {
            spawnCar(gen);
        }

        std::cout << "CarManager initialized with " << cars.size() << " initial cars" << std::endl;
    }
    
    Car *findNearestCar(sf::Vector2f playerPos, float maxDistance)
    {
        Car *nearest = nullptr;
        float minDist = maxDistance;

        for (auto &car : cars)
        {
            if (!car->isActive || car->isDestroyed)
                continue;

            float dx = car->x - playerPos.x;
            float dy = car->y - playerPos.y;
            float dist = std::sqrt(dx * dx + dy * dy);

            if (dist < minDist)
            {
                minDist = dist;
                nearest = car.get();
            }
        }

        return nearest;
    }

    void setCarPlayerControlled(Car *car, bool controlled)
    {
        if (car)
        {
            car->isActive = !controlled; // Disable AI when player drives
        }
    }
    
    // Metodo per ottenere tutte le auto (per il sistema pedoni)
    const std::vector<std::unique_ptr<Car>>& getAllCars() const {
        return cars;
    }
    
    void update(float deltaTime, MapGenerator &gen, sf::Vector2f playerPos)
    {
        // Update existing cars
        std::vector<Car *> carPtrs;
        carPtrs.reserve(cars.size());
        for (auto &car : cars)
        {
            carPtrs.push_back(car.get());
        }

        for (auto &car : cars)
        {
            car->update(deltaTime, carPtrs, playerPos);
        }

        // More frequent spawning to maintain heavy traffic
        spawnTimer += deltaTime;
        if (spawnTimer > 2.0f) // Spawn every 2 seconds instead of 5
        {
            spawnTimer = 0.f;

            int activeCars = 0;
            for (auto &car : cars)
            {
                if (car->isActive)
                    activeCars++;
            }

            // Multiple spawning to fill quickly
            if (activeCars < MAX_CARS)
            {
                int spawnCount = std::min(3, MAX_CARS - activeCars); // Spawn up to 3 cars at once
                for (int i = 0; i < spawnCount; i++)
                {
                    spawnCar(gen);
                }
            }
        }

        // Remove cars that are too far away
        cars.erase(std::remove_if(cars.begin(), cars.end(),
                                  [playerPos](const std::unique_ptr<Car> &car)
                                  {
                                      return car->distanceToPlayer > VIEW_DISTANCE * 3.0f;
                                  }),
                   cars.end());
    }

    void spawnCar(MapGenerator &gen)
    {
        const auto &roadPoints = roadNetwork.getRoadPoints();
        const auto &intersections = roadNetwork.getIntersections();

        if (roadPoints.empty())
        {
            std::cout << "ERROR: No road points available for spawning!" << std::endl;
            return;
        }

        // Spawn on both normal roads and intersections for variety
        std::vector<RoadPoint *> allSpawnPoints;
        allSpawnPoints.reserve(roadPoints.size() + intersections.size());

        // Add all normal roads
        for (auto &road : roadPoints)
        {
            if (!road->isIntersection)
                allSpawnPoints.push_back(road.get());
        }

        // Add some intersections (20% of cases)
        std::uniform_real_distribution<float> intersectionChance(0.f, 1.f);
        if (intersectionChance(rng) < 0.2f)
        {
            for (auto &intersection : intersections)
            {
                allSpawnPoints.push_back(intersection.get());
            }
        }

        if (allSpawnPoints.empty())
        {
            std::cout << "No valid spawn points found!" << std::endl;
            return;
        }

        // Try to spawn avoiding crowded areas
        for (int attempts = 0; attempts < 50; attempts++)
        {
            std::uniform_int_distribution<size_t> pointDist(0, allSpawnPoints.size() - 1);
            RoadPoint *spawnPoint = allSpawnPoints[pointDist(rng)];

            if (!spawnPoint)
                continue;

            // Check if area is already too crowded
            bool tooManyNearby = false;
            int nearbyCount = 0;
            for (auto &car : cars)
            {
                if (!car->isActive)
                    continue;

                float dx = car->x - spawnPoint->worldX;
                float dy = car->y - spawnPoint->worldY;
                float dist = std::sqrt(dx * dx + dy * dy);

                if (dist < 200.0f) // Check area
                {
                    nearbyCount++;
                    if (nearbyCount >= 4) // Max 4 cars in 200px area
                    {
                        tooManyNearby = true;
                        break;
                    }
                }
            }

            if (tooManyNearby)
                continue;

            // Determine direction based on road type
            Direction dir = NONE;

            if (spawnPoint->isIntersection)
            {
                // For intersections, choose random valid direction
                std::vector<Direction> validDirs;
                if (spawnPoint->canGoNorth)
                    validDirs.push_back(NORTH);
                if (spawnPoint->canGoSouth)
                    validDirs.push_back(SOUTH);
                if (spawnPoint->canGoEast)
                    validDirs.push_back(EAST);
                if (spawnPoint->canGoWest)
                    validDirs.push_back(WEST);

                if (!validDirs.empty())
                {
                    std::uniform_int_distribution<size_t> dirDist(0, validDirs.size() - 1);
                    dir = validDirs[dirDist(rng)];
                }
            }
            else
            {
                // For normal roads
                if (gen.isHorizontalRoad(spawnPoint->gridX, spawnPoint->gridY))
                {
                    dir = (std::uniform_int_distribution<int>(0, 1)(rng) == 0) ? EAST : WEST;
                }
                else if (gen.isVerticalRoad(spawnPoint->gridX, spawnPoint->gridY))
                {
                    dir = (std::uniform_int_distribution<int>(0, 1)(rng) == 0) ? NORTH : SOUTH;
                }
            }

            if (dir != NONE)
            {
                auto newCar = std::make_unique<Car>(
                    spawnPoint->worldX, spawnPoint->worldY, dir,
                    &rng, &roadNetwork, &gen);
                cars.push_back(std::move(newCar));
                return;
            }
        }

        // Fallback: simple spawn if intelligent method fails
        for (int attempts = 0; attempts < 20; attempts++)
        {
            if (roadPoints.empty())
                break;

            std::uniform_int_distribution<size_t> roadDist(0, roadPoints.size() - 1);
            RoadPoint *spawnPoint = roadPoints[roadDist(rng)].get();

            if (!spawnPoint || spawnPoint->isIntersection)
                continue;

            Direction dir = NONE;
            if (gen.isHorizontalRoad(spawnPoint->gridX, spawnPoint->gridY))
            {
                dir = (std::uniform_int_distribution<int>(0, 1)(rng) == 0) ? EAST : WEST;
            }
            else if (gen.isVerticalRoad(spawnPoint->gridX, spawnPoint->gridY))
            {
                dir = (std::uniform_int_distribution<int>(0, 1)(rng) == 0) ? NORTH : SOUTH;
            }

            if (dir != NONE)
            {
                auto newCar = std::make_unique<Car>(
                    spawnPoint->worldX, spawnPoint->worldY, dir,
                    &rng, &roadNetwork, &gen);
                cars.push_back(std::move(newCar));
                return;
            }
        }
    }

    void draw(sf::RenderWindow &window)
    {
        for (auto &car : cars)
        {
            car->draw(window);
        }
    }

    void drawNetwork(sf::RenderWindow &window)
    {
        roadNetwork.drawNetwork(window);
    }

    size_t getActiveCarCount() const
    {
        size_t count = 0;
        for (auto &car : cars)
        {
            if (car->isActive)
                count++;
        }
        return count;
    }

    size_t getTotalCarCount() const
    {
        return cars.size();
    }
};

// Texture loading function
void loadTextures()
{
    // Vehicle textures
    hasBusTexture = busTexture.loadFromFile("./assets/bus.png");
    if (!hasBusTexture)
        std::cout << "Warning: bus texture not loaded\n";
    hasSedanTexture = sedanTexture.loadFromFile("./assets/sedan.png");
    if (!hasSedanTexture)
        std::cout << "Warning: sedan texture not loaded\n";
    hasSuvTexture = suvTexture.loadFromFile("./assets/suv.png");
    if (!hasSuvTexture)
        std::cout << "Warning: suv texture not loaded\n";
    hasSportsCarTexture = sportsCarTexture.loadFromFile("./assets/sports_car.png");
    if (!hasSportsCarTexture)
        std::cout << "Warning: sports car texture not loaded\n";
    hasTruckTexture = truckTexture.loadFromFile("./assets/truck.png");
    if (!hasTruckTexture)
        std::cout << "Warning: truck texture not loaded\n";
    hasCompactTexture = compactTexture.loadFromFile("./assets/compact.png");
    if (!hasCompactTexture)
        std::cout << "Warning: compact texture not loaded\n";
    
    // Environment textures
    hasStreetTexture = streetTexture.loadFromFile("./assets/street.png");
    if (!hasStreetTexture)
        std::cout << "Warning: street texture not loaded\n";
    hasSidewalkTexture = sidewalkTexture.loadFromFile("./assets/sidewalk.png");
    if (!hasSidewalkTexture)
        std::cout << "Warning: sidewalk texture not loaded\n";
    hasBuildingTexture = buildingTexture.loadFromFile("./assets/building.png");
    if (!hasBuildingTexture)
        std::cout << "Warning: building texture not loaded\n";
    hasParkTexture = parkTexture.loadFromFile("./assets/park.png");
    if (!hasParkTexture)
        std::cout << "Warning: park texture not loaded\n";
    hasParkingTexture = parkingTexture.loadFromFile("./assets/parking.png");
    if (!hasParkingTexture)
        std::cout << "Warning: parking texture not loaded\n";
    
    // Player textures
    hasPlayerIdle = PlayerTextureIdle.loadFromFile("./assets/player_idle.png");
    if (!hasPlayerIdle)
        std::cout << "Warning: player idle texture not loaded\n";
    hasPlayerWalk = PlayerTextureWalk.loadFromFile("./assets/player_moving.png");
    if (!hasPlayerWalk)
        std::cout << "Warning: player walk texture not loaded\n";
    hasPlayerShoot = PlayerTextureShoot.loadFromFile("./assets/player_shooting.png");
    if (!hasPlayerShoot)
        std::cout << "Warning: player shoot texture not loaded\n";
    hasCarTexture1 = CarTexture1.loadFromFile("./assets/car_1.png");
    if (!hasCarTexture1)
        std::cout << "Warning: car texture not loaded\n";
    
    // Pedestrian textures - GTA style pixel art
    hasPedestrianCitizen = pedestrianCitizenTexture.loadFromFile("./assets/ped_citizen.png");
    if (!hasPedestrianCitizen)
        std::cout << "Warning: citizen pedestrian texture not loaded\n";
    
    hasPedestrianBusinessman = pedestrianBusinessmanTexture.loadFromFile("./assets/ped_businessman.png");
    if (!hasPedestrianBusinessman)
        std::cout << "Warning: businessman pedestrian texture not loaded\n";
    
    hasPedestrianElderly = pedestrianElderlyTexture.loadFromFile("./assets/ped_elderly.png");
    if (!hasPedestrianElderly)
        std::cout << "Warning: elderly pedestrian texture not loaded\n";
    
    hasPedestrianChild = pedestrianChildTexture.loadFromFile("./assets/ped_child.png");
    if (!hasPedestrianChild)
        std::cout << "Warning: child pedestrian texture not loaded\n";
    
    hasPedestrianJogger = pedestrianJoggerTexture.loadFromFile("./assets/ped_jogger.png");
    if (!hasPedestrianJogger)
        std::cout << "Warning: jogger pedestrian texture not loaded\n";
    
    hasPedestrianWoman = pedestrianWomanTexture.loadFromFile("./assets/ped_woman.png");
    if (!hasPedestrianWoman)
        std::cout << "Warning: woman pedestrian texture not loaded\n";
    
    hasPedestrianPolice = pedestrianPoliceTexture.loadFromFile("./assets/ped_police.png");
    if (!hasPedestrianPolice)
        std::cout << "Warning: police pedestrian texture not loaded\n";
    
    std::cout << "Texture loading complete. Pedestrian types available: " 
              << (hasPedestrianCitizen + hasPedestrianBusinessman + hasPedestrianElderly + 
                  hasPedestrianChild + hasPedestrianJogger + hasPedestrianWoman + hasPedestrianPolice) 
              << "/7\n";
}

void drawMap(sf::RenderWindow &window, MapGenerator &gen)
{
    sf::View currentView = window.getView();
    sf::Vector2f center = currentView.getCenter();
    sf::Vector2f size = currentView.getSize();

    int startX = std::max(0, static_cast<int>((center.x - size.x / 2) / TILE_SIZE) - 1);
    int endX = std::min(MAP_SIZE, static_cast<int>((center.x + size.x / 2) / TILE_SIZE) + 2);
    int startY = std::max(0, static_cast<int>((center.y - size.y / 2) / TILE_SIZE) - 1);
    int endY = std::min(MAP_SIZE, static_cast<int>((center.y + size.y / 2) / TILE_SIZE) + 2);

    for (int i = startX; i < endX; i++)
    {
        for (int j = startY; j < endY; j++)
        {
            int cell = gen.getCell(i, j);

            sf::RectangleShape rect;
            rect.setSize(sf::Vector2f(static_cast<float>(TILE_SIZE), static_cast<float>(TILE_SIZE)));
            rect.setPosition(sf::Vector2f(static_cast<float>(i * TILE_SIZE), static_cast<float>(j * TILE_SIZE)));

            switch (cell)
            {
            case STREET:
                if (hasStreetTexture)
                    rect.setTexture(&streetTexture);
                else
                    rect.setFillColor(sf::Color(80, 80, 80));
                break;
            case SIDEWALK:
                if (hasSidewalkTexture)
                    rect.setTexture(&sidewalkTexture);
                else
                    rect.setFillColor(sf::Color(180, 180, 180));
                break;
            case BUILDING:
                if (hasBuildingTexture)
                    rect.setTexture(&buildingTexture);
                else
                    rect.setFillColor(sf::Color(100, 100, 150));
                break;
            case PARK:
                if (hasParkTexture)
                    rect.setTexture(&parkTexture);
                else
                    rect.setFillColor(sf::Color(50, 150, 50));
                break;
            case PARKING:
                if (hasParkingTexture)
                    rect.setTexture(&parkingTexture);
                else
                    rect.setFillColor(sf::Color(150, 150, 100));
                break;
            default:
                continue;
            }

            window.draw(rect);
        }
    }
}

// Player class with improved car hijacking
struct Player
{
    sf::CircleShape shape;
    float x, y;
    float speed;
    bool isMoving;
    float rotation;
    bool isDriving = false;
    Car *currentCar = nullptr;
    float carHijackRange = 40.f;

    Player(float startX, float startY)
        : x(startX), y(startY), speed(120.0f), isMoving(false), rotation(0.0f) // Ridotto da 150.0f
    {
        shape.setRadius(15.f);
        shape.setFillColor(sf::Color::Red);
        shape.setOrigin(sf::Vector2f(15.f, 15.f));

        if (hasPlayerIdle)
        {
            shape.setTexture(&PlayerTextureIdle);
        }

        updatePosition();
    }

    void update(float deltaTime, MapGenerator &gen, CarManager *carManager)
    {
        if (isDriving && currentCar)
        {
            if (currentCar->isDestroyed) {
                // Car destroyed, exit automatically
                carManager->setCarPlayerControlled(currentCar, false);
                if (hasPlayerIdle) shape.setTexture(&PlayerTextureIdle);
                std::cout << "Car exploded! You were ejected!" << std::endl;
                isDriving = false;
                currentCar = nullptr;
                return;
            }
            
            // Velocità auto ridotta e scalata con danno
            float baseCarSpeed = 250.0f; // Ridotto da 500.0f
            float carSpeed = baseCarSpeed * currentCar->getHealthPercentage();
            float dx = 0.f, dy = 0.f;

            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W))
                dy = -carSpeed * deltaTime;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S))
                dy = carSpeed * deltaTime;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A))
                dx = -carSpeed * deltaTime;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D))
                dx = carSpeed * deltaTime;

            // Calcola velocità attuale per sistema danni
            float currentSpeed = std::sqrt(dx*dx + dy*dy) / deltaTime;
            
            // NUOVO SISTEMA DI COLLISIONI SEMPLIFICATO
            bool canMoveX = true;
            bool canMoveY = true;
            
            // Test movimento X usando la nuova funzione
            if (dx != 0.f) {
                float testX = currentCar->x + dx;
                canMoveX = !currentCar->checkBuildingCollision(testX, currentCar->y, gen);
            }
            
            // Test movimento Y usando la nuova funzione  
            if (dy != 0.f) {
                float testY = currentCar->y + dy;
                canMoveY = !currentCar->checkBuildingCollision(currentCar->x, testY, gen);
            }
            
            // Applica movimento con sistema danni migliorato
            bool hasCollision = (!canMoveX && dx != 0.f) || (!canMoveY && dy != 0.f);
            
            float actualDx = canMoveX ? dx : 0.f;
            float actualDy = canMoveY ? dy : 0.f;
            
            // Sistema danni con cooldown
            currentCar->updateCollisionDamage(deltaTime, currentSpeed, hasCollision, sf::Vector2f(dx, dy));
            
            // Effetto rimbalzo leggero solo per collisioni ad alta velocità
            if (hasCollision && currentSpeed > 150.0f) {
                if (!canMoveX && dx != 0.f) {
                    actualDx = -dx * 0.05f; // Rimbalzo molto ridotto
                }
                if (!canMoveY && dy != 0.f) {
                    actualDy = -dy * 0.05f;
                }
            }
            
            // Aggiorna posizione
            currentCar->x += actualDx;
            currentCar->y += actualDy;
            currentCar->carShape.setPosition(sf::Vector2f(currentCar->x, currentCar->y));
            
            // Update car rotation
            updateCarRotation(actualDx, actualDy, deltaTime);
            
            // Player segue l'auto
            x = currentCar->x;
            y = currentCar->y;
        }
        else
        {
            // Normal player mode (unchanged)
            float dx = 0.f, dy = 0.f;
            bool wasMoving = isMoving;
            isMoving = false;

            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W))
            {
                dy = -speed * deltaTime;
                rotation = 90.f;
                isMoving = true;
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S))
            {
                dy = speed * deltaTime;
                rotation = 270.f;
                isMoving = true;
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A))
            {
                dx = -speed * deltaTime;
                rotation = 0.f;
                isMoving = true;
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D))
            {
                dx = speed * deltaTime;
                rotation = 180.f;
                isMoving = true;
            }

            // Check collisions before moving (avoid buildings)
            float newX = x + dx;
            float newY = y + dy;

            int gridX = static_cast<int>(newX / TILE_SIZE);
            int gridY = static_cast<int>(newY / TILE_SIZE);

            if (gen.getCell(gridX, gridY) != BUILDING)
            {
                x = newX;
                y = newY;
            }

            // Update texture based on movement
            if (isMoving != wasMoving)
            {
                if (isMoving && hasPlayerWalk)
                {
                    shape.setTexture(&PlayerTextureWalk);
                }
                else if (!isMoving && hasPlayerIdle)
                {
                    shape.setTexture(&PlayerTextureIdle);
                }
            }
        }

        updatePosition();
    }

    void updateCarRotation(float dx, float dy, float deltaTime) {
        if (!currentCar) return;
        
        float healthPct = currentCar->getHealthPercentage();
        float rotationOffset = 0.f;
        
        if (healthPct < 0.3f) {
            // Very damaged car, unstable controls
            static float timeCounter = 0.f;
            timeCounter += deltaTime;
            rotationOffset = (std::sin(timeCounter * 10) * 5.f) * (1.f - healthPct);
        }
        
        // Rotazione solo se c'è movimento effettivo
        if (std::abs(dx) > 0.1f || std::abs(dy) > 0.1f) {
            if (std::abs(dx) > std::abs(dy)) {
                // Movimento orizzontale prevalente
                if (dx > 0.1f)
                    currentCar->carShape.setRotation(sf::degrees(0 + rotationOffset));
                else if (dx < -0.1f)
                    currentCar->carShape.setRotation(sf::degrees(180 + rotationOffset));
            } else {
                // Movimento verticale prevalente
                if (dy < -0.1f)
                    currentCar->carShape.setRotation(sf::degrees(270 + rotationOffset));
                else if (dy > 0.1f)
                    currentCar->carShape.setRotation(sf::degrees(90 + rotationOffset));
            }
        }
    }
    
    void updatePosition()
    {
        shape.setPosition(sf::Vector2f(x, y));
        shape.setRotation(sf::degrees(rotation));
    }

    sf::Vector2f getPosition() const
    {
        return sf::Vector2f(x, y);
    }

    void draw(sf::RenderWindow &window)
    {
        if (!isDriving) // Draw player only if not driving
        {
            window.draw(shape);
        }
    }
};

// Main function
int main()
{
    sf::RenderWindow window(sf::VideoMode({1200, 800}), "GTA-Style Enhanced Game (SFML 3.0)");
    window.setFramerateLimit(60);

    loadTextures();
    
    // Inizializza il database delle dimensioni veicoli
    VehicleDatabase::initializeDimensions();

    MapGenerator gen;
    gen.generateGridCity();

    CarManager carManager;
    carManager.initialize(gen);
    
    // Inizializza il sistema pedoni
    PedestrianManager pedestrianManager;
    pedestrianManager.initialize(gen);

    Player player(MAP_SIZE * TILE_SIZE / 2.f, MAP_SIZE * TILE_SIZE / 2.f);

    sf::View camera;
    camera.setSize(sf::Vector2f(1200.f, 800.f));
    camera.setCenter(player.getPosition());

    sf::Clock clock;
    bool showDebug = false;

    while (window.isOpen())
    {
        float deltaTime = clock.restart().asSeconds();

        // Event loop SFML 3 style (std::optional)
        while (auto event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
                window.close();

            if (event->is<sf::Event::KeyPressed>())
            {
                auto kp = event->getIf<sf::Event::KeyPressed>();
                if (kp && kp->code == sf::Keyboard::Key::Tab)
                {
                    showDebug = !showDebug;
                }

                // F key handling for car theft/exit
                if (kp && kp->code == sf::Keyboard::Key::F)
                {
                    if (!player.isDriving)
                    {
                        // Look for nearby car to steal
                        Car *nearestCar = carManager.findNearestCar(player.getPosition(), player.carHijackRange);
                        if (nearestCar)
                        {
                            player.isDriving = true;
                            player.currentCar = nearestCar;
                            carManager.setCarPlayerControlled(nearestCar, true);

                            // Teleport player to car
                            player.x = nearestCar->x;
                            player.y = nearestCar->y;

                            std::cout << "Stole " << nearestCar->getVehicleTypeName() << "!\n";
                        }
                        else
                        {
                            std::cout << "No nearby car to steal!\n";
                        }
                    }
                    else
                    {
                        // Exit car
                        if (player.currentCar)
                        {
                            carManager.setCarPlayerControlled(player.currentCar, false);

                            // Player reappears
                            if (hasPlayerIdle)
                                player.shape.setTexture(&PlayerTextureIdle);

                            std::cout << "Exited " << player.currentCar->getVehicleTypeName() << "\n";
                        }

                        player.isDriving = false;
                        player.currentCar = nullptr;
                    }
                }
            }
        }

        // Update game state
        player.update(deltaTime, gen, &carManager);
        carManager.update(deltaTime, gen, player.getPosition());
        
        // Update pedestrians
        pedestrianManager.update(deltaTime, player.getPosition(), carManager.getAllCars(), gen);

        // Camera follows player
        camera.setCenter(player.getPosition());
        window.setView(camera);

        // Render everything
        window.clear(sf::Color(40, 40, 40));

        drawMap(window, gen);

        if (showDebug)
        {
            carManager.drawNetwork(window);
        }

        carManager.draw(window);
        pedestrianManager.draw(window); // Draw pedestrians
        player.draw(window);

        // UI Info (use default view)
        sf::View uiView = window.getDefaultView();
        window.setView(uiView);

        // Draw UI elements (simplified for SFML 3.0 compatibility)
        sf::Font font;
        bool hasFont = false;
        
        // Try to load font, but continue even if it fails
        try {
            hasFont = font.openFromFile("./assets/arial.ttf");
        } catch (...) {
            hasFont = false;
        }
        
        if (hasFont)
        {
            // Draw car count info
            sf::Text carCountText(font);
            carCountText.setCharacterSize(20);
            carCountText.setFillColor(sf::Color::White);
            carCountText.setPosition(sf::Vector2f(10, 10));
            carCountText.setString("Cars: " + std::to_string(carManager.getActiveCarCount()) + 
                                  " | Pedestrians: " + std::to_string(pedestrianManager.getActivePedestrianCount()));
            window.draw(carCountText);

            // Draw player status
            sf::Text statusText(font);
            statusText.setCharacterSize(18);
            statusText.setFillColor(sf::Color::Yellow);
            statusText.setPosition(sf::Vector2f(10, 40));
            
            if (player.isDriving && player.currentCar)
            {
                std::string healthInfo = "Driving: " + player.currentCar->getVehicleTypeName() + 
                                       " | Health: " + std::to_string((int)player.currentCar->health) + 
                                       "/" + std::to_string((int)player.currentCar->maxHealth);
                if (player.currentCar->isOnFire)
                    healthInfo += " [ON FIRE!]";
                if (player.currentCar->isDestroyed)
                    healthInfo += " [DESTROYED]";
                    
                statusText.setString(healthInfo);
            }
            else
            {
                statusText.setString("On foot - Press F near a car to steal it");
            }
            window.draw(statusText);

            // Draw controls help
            sf::Text helpText(font);
            helpText.setCharacterSize(16);
            helpText.setFillColor(sf::Color::Cyan);
            helpText.setPosition(sf::Vector2f(10, 70));
            helpText.setString("Controls: WASD=Move/Drive | F=Steal/Exit Car | Tab=Debug View");
            window.draw(helpText);

            // Draw performance info
            sf::Text perfText(font);
            perfText.setCharacterSize(14);
            perfText.setFillColor(sf::Color::Green);
            perfText.setPosition(sf::Vector2f(10, 100));
            perfText.setString("FPS: " + std::to_string((int)(1.0f / deltaTime)) + 
                             " | Total Cars: " + std::to_string(carManager.getTotalCarCount()) +
                             " | Total Pedestrians: " + std::to_string(pedestrianManager.getTotalPedestrianCount()));
            window.draw(perfText);

            // Draw damage warning if in damaged car
            if (player.isDriving && player.currentCar && player.currentCar->getHealthPercentage() < 0.3f)
            {
                sf::Text warningText(font);
                warningText.setCharacterSize(24);
                warningText.setFillColor(sf::Color::Red);
                warningText.setPosition(sf::Vector2f(window.getSize().x / 2 - 150, 50));
                warningText.setString("WARNING: VEHICLE CRITICALLY DAMAGED!");
                
                // Flashing effect
                static float flashTimer = 0.f;
                flashTimer += deltaTime;
                if (std::sin(flashTimer * 10) > 0)
                {
                    window.draw(warningText);
                }
            }
        }

        window.display();
    }

    return 0;
}