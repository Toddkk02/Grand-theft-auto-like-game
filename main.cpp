// main.cpp - Complete GTA-style game with car damage system (SFML 3.0 compatible)
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

// Game constants
const int MAP_SIZE = 100;
const int TILE_SIZE = 64;
const int BLOCK_SIZE = 8;
const float BASE_CAR_SPEED = 120.0f;
const float VIEW_DISTANCE = 1200.0f;
const int MAX_CARS = 80;

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
static bool hasSedanTexture = false;
static bool hasSuvTexture = false;
static bool hasSportsCarTexture = false;
static bool hasTruckTexture = false;
static bool hasCompactTexture = false;
static bool hasBusTexture = false;

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

// Vehicle properties database
class VehicleDatabase
{
public:
    static VehicleProperties getRandomVehicleProperties(std::mt19937 &rng)
    {
        std::uniform_int_distribution<int> typeDist(0, 5);
        VehicleType type = static_cast<VehicleType>(typeDist(rng));

        switch (type)
        {
        case SEDAN:
            return VehicleProperties(
                SEDAN,
                sf::Vector2f(45.f, 25.f),
                (hasSedanTexture ? &sedanTexture : nullptr),
                1.0f, 0.3f, 0.8f);

        case SUV:
            return VehicleProperties(
                SUV,
                sf::Vector2f(50.f, 30.f),
                (hasSuvTexture ? &suvTexture : nullptr),
                0.85f, 0.2f, 0.6f);

        case SPORTS_CAR:
            return VehicleProperties(
                SPORTS_CAR,
                sf::Vector2f(42.f, 22.f),
                (hasSportsCarTexture ? &sportsCarTexture : nullptr),
                1.4f, 0.8f, 1.2f);

        case TRUCK:
            return VehicleProperties(
                TRUCK,
                sf::Vector2f(60.f, 35.f),
                (hasTruckTexture ? &truckTexture : nullptr),
                0.7f, 0.1f, 0.4f);

        case COMPACT:
            return VehicleProperties(
                COMPACT,
                sf::Vector2f(38.f, 22.f),
                (hasCompactTexture ? &compactTexture : nullptr),
                1.1f, 0.4f, 1.0f);

        case BUS:
            return VehicleProperties(
                BUS,
                sf::Vector2f(70.f, 30.f),
                (hasBusTexture ? &busTexture : nullptr),
                0.6f, 0.0f, 0.3f);

        default:
            return VehicleProperties(
                SEDAN,
                sf::Vector2f(45.f, 25.f),
                (hasSedanTexture ? &sedanTexture : nullptr),
                1.0f, 0.3f, 0.8f);
        }
    }
};

// Forward declarations
struct MapGenerator;
struct Car;

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

// Player class with car hijacking
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
        : x(startX), y(startY), speed(150.0f), isMoving(false), rotation(0.0f)
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
        
        // Stolen car driving mode
        float carSpeed = 500.0f * currentCar->getHealthPercentage(); // Reduced speed if damaged
        float dx = 0.f, dy = 0.f;

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W))
            dy = -carSpeed * deltaTime;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S))
            dy = carSpeed * deltaTime;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A))
            dx = -carSpeed * deltaTime;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D))
            dx = carSpeed * deltaTime;

        // Save previous position for speed calculation
        sf::Vector2f previousPos(currentCar->x, currentCar->y);
        
        // Calculate new position
        float newX = currentCar->x + dx;
        float newY = currentCar->y + dy;
        
        // SISTEMA DI COLLISIONI MIGLIORATO
        bool canMoveX = true;
        bool canMoveY = true;
        float currentSpeed = std::sqrt(dx*dx + dy*dy) / deltaTime;
        
        // Controlla multiple celle intorno alla macchina per collisioni più accurate
        sf::Vector2f carSize = currentCar->properties.size;
        float halfWidth = carSize.x / 2.0f;
        float halfHeight = carSize.y / 2.0f;
        
        // Punti di controllo intorno alla macchina
        std::vector<sf::Vector2f> checkPoints = {
            {newX - halfWidth, newY - halfHeight}, // Top-left
            {newX + halfWidth, newY - halfHeight}, // Top-right
            {newX - halfWidth, newY + halfHeight}, // Bottom-left
            {newX + halfWidth, newY + halfHeight}, // Bottom-right
            {newX, newY - halfHeight},             // Top-center
            {newX, newY + halfHeight},             // Bottom-center
            {newX - halfWidth, newY},              // Left-center
            {newX + halfWidth, newY}               // Right-center
        };
        
        // Controllo collisioni sull'asse X
        if (dx != 0.f) {
            std::vector<sf::Vector2f> xCheckPoints = {
                {newX + (dx > 0 ? halfWidth : -halfWidth), currentCar->y - halfHeight},
                {newX + (dx > 0 ? halfWidth : -halfWidth), currentCar->y},
                {newX + (dx > 0 ? halfWidth : -halfWidth), currentCar->y + halfHeight}
            };
            
            for (const auto& point : xCheckPoints) {
                int gridX = static_cast<int>(point.x / TILE_SIZE);
                int gridY = static_cast<int>(point.y / TILE_SIZE);
                
                if (gen.getCell(gridX, gridY) == BUILDING) {
                    canMoveX = false;
                    break;
                }
            }
        }
        
        // Controllo collisioni sull'asse Y
        if (dy != 0.f) {
            std::vector<sf::Vector2f> yCheckPoints = {
                {currentCar->x - halfWidth, newY + (dy > 0 ? halfHeight : -halfHeight)},
                {currentCar->x, newY + (dy > 0 ? halfHeight : -halfHeight)},
                {currentCar->x + halfWidth, newY + (dy > 0 ? halfHeight : -halfHeight)}
            };
            
            for (const auto& point : yCheckPoints) {
                int gridX = static_cast<int>(point.x / TILE_SIZE);
                int gridY = static_cast<int>(point.y / TILE_SIZE);
                
                if (gen.getCell(gridX, gridY) == BUILDING) {
                    canMoveY = false;
                    break;
                }
            }
        }
        
        // Applica movimento solo se possibile
        float actualDx = canMoveX ? dx : 0.f;
        float actualDy = canMoveY ? dy : 0.f;
        
        // Se c'è stata una collisione e la velocità è significativa, applica danni
        if ((!canMoveX && dx != 0.f) || (!canMoveY && dy != 0.f)) {
            // Calcola la severità della collisione
            float impactSpeed = currentSpeed;
            
            // Solo danni se la velocità è sopra una soglia minima
            if (impactSpeed > 100.0f) { // Soglia minima per danni
                float damage = (impactSpeed - 100.0f) * 0.3f; // Danni graduali
                
                // Limita i danni massimi per impatto
                damage = std::min(damage, 150.0f);
                
                currentCar->takeDamage(damage);
                
                std::cout << "CRASH! Speed: " << (int)impactSpeed 
                          << " | Damage: " << (int)damage 
                          << " | Health: " << (int)currentCar->health << "/" << (int)currentCar->maxHealth << std::endl;
                
                // Effetto di rimbalzo leggero
                if (!canMoveX && dx != 0.f) {
                    actualDx = -dx * 0.1f; // Piccolo rimbalzo
                }
                if (!canMoveY && dy != 0.f) {
                    actualDy = -dy * 0.1f; // Piccolo rimbalzo
                }
            }
        }
        
        // Aggiorna posizione della macchina
        currentCar->x += actualDx;
        currentCar->y += actualDy;
        currentCar->carShape.setPosition(sf::Vector2f(currentCar->x, currentCar->y));
        
        // Aggiorna velocità per il sistema di danno
        currentCar->lastSpeed = std::sqrt(actualDx*actualDx + actualDy*actualDy) / deltaTime;
        currentCar->lastPosition = previousPos;
        
        // Update car rotation (with shake if heavily damaged)
        float healthPct = currentCar->getHealthPercentage();
        float rotationOffset = 0.f;
        
        if (healthPct < 0.3f) {
            // Very damaged car, unstable controls
            static float timeCounter = 0.f;
            timeCounter += deltaTime;
            rotationOffset = (std::sin(timeCounter * 10) * 5.f) * (1.f - healthPct);
        }
        
        // Rotazione solo se c'è movimento effettivo
        if (actualDx > 0.1f)
            currentCar->carShape.setRotation(sf::degrees(0 + rotationOffset));
        else if (actualDx < -0.1f)
            currentCar->carShape.setRotation(sf::degrees(180 + rotationOffset));
        else if (actualDy < -0.1f)
            currentCar->carShape.setRotation(sf::degrees(270 + rotationOffset));
        else if (actualDy > 0.1f)
            currentCar->carShape.setRotation(sf::degrees(90 + rotationOffset));

        // Player position follows car
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
    sf::RenderWindow window(sf::VideoMode({1200, 800}), "GTA-Style Traffic System (SFML 3.0)");
    window.setFramerateLimit(60);

    loadTextures();

    MapGenerator gen;
    gen.generateGridCity();

    CarManager carManager;
    carManager.initialize(gen);

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
            carCountText.setString("Active Cars: " + std::to_string(carManager.getActiveCarCount()) + 
                                  " / Total: " + std::to_string(carManager.getTotalCarCount()));
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