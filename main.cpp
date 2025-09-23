// main.cpp
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

// Costanti
const int MAP_SIZE = 100;
const int TILE_SIZE = 64;
const int BLOCK_SIZE = 8; // Dimensione blocco città (8x8 tiles)
const float BASE_CAR_SPEED = 120.0f; // Velocità base auto
const float VIEW_DISTANCE = 1200.0f; // Aumentata distanza di vista
const int MAX_CARS = 80; // Molte più macchine!

// Texture globali per diversi tipi di veicoli
static sf::Texture streetTexture;
static sf::Texture sidewalkTexture;
static sf::Texture buildingTexture;
static sf::Texture parkTexture;
static sf::Texture parkingTexture;
static sf::Texture PlayerTextureIdle;
static sf::Texture PlayerTextureWalk;
static sf::Texture PlayerTextureShoot;
static sf::Texture CarTexture1;

// Flags per texture caricate
static bool hasStreetTexture = false;
static bool hasSidewalkTexture = false;
static bool hasBuildingTexture = false;
static bool hasParkTexture = false;
static bool hasParkingTexture = false;
static bool hasPlayerIdle = false;
static bool hasPlayerWalk = false;
static bool hasPlayerShoot = false;
static bool hasCarTexture1 = false;

// Enum per i tipi di veicoli
enum VehicleType
{
    SEDAN = 0,
    SUV = 1,
    SPORTS_CAR = 2,
    TRUCK = 3,
    COMPACT = 4,
    BUS = 5
};

// Enum per i tipi di celle
enum MapType
{
    NOTHING = 0,
    STREET = 1,
    BUILDING = 2,
    PARK = 3,
    PARKING = 4,
    SIDEWALK = 5,
};

// Enum per le direzioni (coordinato con rotazione SFML)
enum Direction
{
    NORTH = 0,   // Su
    EAST = 1,    // Destra  
    SOUTH = 2,   // Giù
    WEST = 3,    // Sinistra
    NONE = -1
};

// Struttura per le proprietà dei veicoli
struct VehicleProperties
{
    VehicleType type;
    sf::Vector2f size;
    sf::Color color;
    float speedMultiplier;
    float aggressiveness; // 0.0 = molto calmo, 1.0 = molto aggressivo
    float turnRate; // Velocità di svolta agli incroci
    
    VehicleProperties() = default;
    VehicleProperties(VehicleType t, sf::Vector2f s, sf::Color c, float speed, float aggr, float turn)
        : type(t), size(s), color(c), speedMultiplier(speed), aggressiveness(aggr), turnRate(turn) {}
};

// Database delle proprietà dei veicoli
class VehicleDatabase
{
public:
    static VehicleProperties getRandomVehicleProperties(std::mt19937& rng)
    {
        std::uniform_int_distribution<int> typeDist(0, 5);
        VehicleType type = static_cast<VehicleType>(typeDist(rng));
        
        switch (type)
        {
            case SEDAN:
                return VehicleProperties(
                    SEDAN,
                    sf::Vector2f(45.f, 25.f),
                    getRandomColor(rng, {sf::Color::White, sf::Color::Black, sf::Color(150, 150, 150), sf::Color(50, 50, 100)}),
                    1.0f, 0.3f, 0.8f
                );
            
            case SUV:
                return VehicleProperties(
                    SUV,
                    sf::Vector2f(50.f, 30.f),
                    getRandomColor(rng, {sf::Color::Black, sf::Color(100, 100, 100), sf::Color(150, 100, 50)}),
                    0.85f, 0.2f, 0.6f
                );
            
            case SPORTS_CAR:
                return VehicleProperties(
                    SPORTS_CAR,
                    sf::Vector2f(42.f, 22.f),
                    getRandomColor(rng, {sf::Color::Red, sf::Color::Yellow, sf::Color::Blue, sf::Color(255, 100, 0)}),
                    1.4f, 0.8f, 1.2f
                );
            
            case TRUCK:
                return VehicleProperties(
                    TRUCK,
                    sf::Vector2f(60.f, 35.f),
                    getRandomColor(rng, {sf::Color(100, 100, 100), sf::Color::White, sf::Color(150, 150, 150)}),
                    0.7f, 0.1f, 0.4f
                );
            
            case COMPACT:
                return VehicleProperties(
                    COMPACT,
                    sf::Vector2f(38.f, 22.f),
                    getRandomColor(rng, {sf::Color::Green, sf::Color::Blue, sf::Color::Cyan, sf::Color::Magenta}),
                    1.1f, 0.4f, 1.0f
                );
            
            case BUS:
                return VehicleProperties(
                    BUS,
                    sf::Vector2f(70.f, 30.f),
                    getRandomColor(rng, {sf::Color::Yellow, sf::Color::Blue, sf::Color::White}),
                    0.6f, 0.0f, 0.3f
                );
            
            default:
                return VehicleProperties(SEDAN, sf::Vector2f(45.f, 25.f), sf::Color::White, 1.0f, 0.3f, 0.8f);
        }
    }

private:
    static sf::Color getRandomColor(std::mt19937& rng, const std::vector<sf::Color>& colors)
    {
        std::uniform_int_distribution<size_t> colorDist(0, colors.size() - 1);
        return colors[colorDist(rng)];
    }
};

// Forward declarations
struct MapGenerator;
struct Car;

// Struttura per rappresentare un punto sulla strada
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

// Rete stradale migliorata
class RoadNetwork
{
private:
    std::vector<std::unique_ptr<RoadPoint>> intersections;
    std::vector<std::unique_ptr<RoadPoint>> roadPoints;
    std::mt19937 rng;
    
public:
    RoadNetwork() : rng(std::random_device{}()) {}
    
    void generateNetwork(MapGenerator& gen);
    RoadPoint* getNearestRoadPoint(float x, float y);
    RoadPoint* getNearestIntersection(float x, float y);
    RoadPoint* getNextRoadPoint(int currentX, int currentY, Direction dir);
    Direction getValidDirections(RoadPoint* point, Direction currentDir);
    bool isValidDirection(RoadPoint* point, Direction dir);
    void drawNetwork(sf::RenderWindow& window);
    
    const std::vector<std::unique_ptr<RoadPoint>>& getIntersections() const { return intersections; }
    const std::vector<std::unique_ptr<RoadPoint>>& getRoadPoints() const { return roadPoints; }
};

// MapGenerator migliorato
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
        // Genera città a griglia perfetta con blocchi 8x8
        for (int i = 0; i < MAP_SIZE; i++)
        {
            for (int j = 0; j < MAP_SIZE; j++)
            {
                int posInBlockX = i % BLOCK_SIZE;
                int posInBlockY = j % BLOCK_SIZE;
                
                // Strade esattamente al centro dei blocchi
                bool isHorizontalRoad = (posInBlockY == 3);
                bool isVerticalRoad = (posInBlockX == 3);
                
                if (isHorizontalRoad || isVerticalRoad)
                {
                    map[i][j] = STREET;
                }
                else if (posInBlockY == 2 || posInBlockY == 4 || 
                         posInBlockX == 2 || posInBlockX == 4)
                {
                    // Marciapiedi accanto alle strade
                    map[i][j] = SIDEWALK;
                }
                else
                {
                    // Riempi con edifici/parchi
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
        
        // Deve essere strada E al centro esatto del blocco
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
        
        // Incrocio = dove si incontrano strada orizzontale E verticale
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

// Implementazione RoadNetwork migliorata
void RoadNetwork::generateNetwork(MapGenerator& gen)
{
    intersections.clear();
    roadPoints.clear();
    
    // Trova tutti i punti stradali
    for (int i = 0; i < MAP_SIZE; i++)
    {
        for (int j = 0; j < MAP_SIZE; j++)
        {
            if (gen.isRoadCenter(i, j))
            {
                if (gen.isIntersection(i, j))
                {
                    auto inter = std::make_unique<RoadPoint>(i, j, true);
                    
                    // Controlla le direzioni possibili
                    inter->canGoNorth = gen.isRoadCenter(i, j - 1);
                    inter->canGoSouth = gen.isRoadCenter(i, j + 1);
                    inter->canGoEast = gen.isRoadCenter(i + 1, j);
                    inter->canGoWest = gen.isRoadCenter(i - 1, j);
                    
                    intersections.push_back(std::move(inter));
                }
                else
                {
                    // Punto strada normale
                    auto road = std::make_unique<RoadPoint>(i, j, false);
                    roadPoints.push_back(std::move(road));
                }
            }
        }
    }
    
    std::cout << "Generated " << intersections.size() << " intersections and " 
              << roadPoints.size() << " road points\n";
}

RoadPoint* RoadNetwork::getNearestRoadPoint(float x, float y)
{
    RoadPoint* nearest = nullptr;
    float minDist = std::numeric_limits<float>::max();
    
    // Controlla prima gli incroci
    for (auto& inter : intersections)
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
    
    // Poi i punti strada normali
    for (auto& road : roadPoints)
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

RoadPoint* RoadNetwork::getNearestIntersection(float x, float y)
{
    RoadPoint* nearest = nullptr;
    float minDist = std::numeric_limits<float>::max();
    
    for (auto& inter : intersections)
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

RoadPoint* RoadNetwork::getNextRoadPoint(int currentX, int currentY, Direction dir)
{
    int nextX = currentX;
    int nextY = currentY;
    
    switch (dir)
    {
        case NORTH: nextY--; break;
        case SOUTH: nextY++; break;
        case EAST: nextX++; break;
        case WEST: nextX--; break;
        default: return nullptr;
    }
    
    // Cerca negli incroci
    for (auto& inter : intersections)
    {
        if (inter->gridX == nextX && inter->gridY == nextY)
            return inter.get();
    }
    
    // Cerca nei punti strada
    for (auto& road : roadPoints)
    {
        if (road->gridX == nextX && road->gridY == nextY)
            return road.get();
    }
    
    return nullptr;
}

Direction RoadNetwork::getValidDirections(RoadPoint* point, Direction currentDir)
{
    if (!point || !point->isIntersection)
        return currentDir; // Se non è un incrocio, continua dritto
    
    std::vector<Direction> validDirs;
    Direction opposite = static_cast<Direction>((currentDir + 2) % 4);
    
    // Aggiungi tutte le direzioni valide eccetto quella opposta
    if (point->canGoNorth && currentDir != opposite)
        validDirs.push_back(NORTH);
    if (point->canGoEast && currentDir != opposite)
        validDirs.push_back(EAST);
    if (point->canGoSouth && currentDir != opposite)
        validDirs.push_back(SOUTH);
    if (point->canGoWest && currentDir != opposite)
        validDirs.push_back(WEST);
    
    if (validDirs.empty())
        return opposite; // Costretto a tornare indietro
    
    // 70% probabilità di continuare dritto se possibile
    if (std::find(validDirs.begin(), validDirs.end(), currentDir) != validDirs.end())
    {
        if (std::uniform_real_distribution<float>(0, 1)(rng) < 0.7f)
            return currentDir;
    }
    
    // Scegli direzione casuale
    std::uniform_int_distribution<size_t> dirDist(0, validDirs.size() - 1);
    return validDirs[dirDist(rng)];
}

bool RoadNetwork::isValidDirection(RoadPoint* point, Direction dir)
{
    if (!point)
        return false;
    
    switch (dir)
    {
        case NORTH: return point->canGoNorth;
        case SOUTH: return point->canGoSouth;
        case EAST: return point->canGoEast;
        case WEST: return point->canGoWest;
        default: return false;
    }
}

void RoadNetwork::drawNetwork(sf::RenderWindow& window)
{
    // Disegna incroci in giallo
    for (auto& inter : intersections)
    {
        sf::CircleShape circle(4.f);
        circle.setFillColor(sf::Color::Yellow);
        circle.setOrigin(sf::Vector2f(4.f, 4.f));
        circle.setPosition(sf::Vector2f(inter->worldX, inter->worldY));
        window.draw(circle);
    }
    
    // Disegna punti strada in verde
    for (auto& road : roadPoints)
    {
        sf::CircleShape circle(2.f);
        circle.setFillColor(sf::Color::Green);
        circle.setOrigin(sf::Vector2f(2.f, 2.f));
        circle.setPosition(sf::Vector2f(road->worldX, road->worldY));
        window.draw(circle);
    }
}

// Struttura Car completamente riscritta con varietà di veicoli
struct Car
{
    sf::RectangleShape carShape;
    float x, y;
    float baseSpeed;
    float currentSpeed;
    Direction currentDirection;
    VehicleProperties properties;
    
    // Stato AI migliorato
    RoadPoint* currentRoadPoint;
    RoadPoint* targetRoadPoint;
    float stopTimer;
    float distanceToPlayer;
    bool isActive;
    bool isStopping;
    
    // Comportamento personalizzato
    float aggressionTimer;
    float honkTimer;
    bool isHonking;
    
    // Grid-based movement
    bool isMovingToTarget;
    float targetX, targetY;
    
    // Random e network
    std::mt19937* rng;
    RoadNetwork* roadNetwork;
    MapGenerator* mapGen;
    
    Car(float startX, float startY, Direction startDir, std::mt19937* randomGen, 
        RoadNetwork* network, MapGenerator* gen)
        : x(startX), y(startY), currentDirection(startDir),
          properties(VehicleDatabase::getRandomVehicleProperties(*randomGen)), // Inizializza qui
          currentRoadPoint(nullptr), targetRoadPoint(nullptr), stopTimer(0.f), 
          distanceToPlayer(0.f), isActive(true), isStopping(false),
          aggressionTimer(0.f), honkTimer(0.f), isHonking(false),
          isMovingToTarget(false), targetX(startX), targetY(startY),
          rng(randomGen), roadNetwork(network), mapGen(gen)
    {
        // Imposta velocità basata sul tipo di veicolo
        baseSpeed = BASE_CAR_SPEED * properties.speedMultiplier;
        currentSpeed = baseSpeed;
        
        // Configura forma basata sulle proprietà
        carShape.setSize(properties.size);
        carShape.setOrigin(sf::Vector2f(properties.size.x / 2.0f, properties.size.y / 2.0f));
        carShape.setPosition(sf::Vector2f(x, y));
        carShape.setFillColor(properties.color);
        
        // Imposta rotazione basata su direzione
        updateRotation();
        
        // Applica texture se disponibile
        if (hasCarTexture1)
        {
            carShape.setTexture(&CarTexture1);
        }
        
        // Trova punto strada corrente e target iniziale
        snapToNearestRoad();
        findNextTarget();
    }
    
    std::string getVehicleTypeName() const
    {
        switch (properties.type)
        {
            case SEDAN: return "Sedan";
            case SUV: return "SUV";
            case SPORTS_CAR: return "Sports Car";
            case TRUCK: return "Truck";
            case COMPACT: return "Compact";
            case BUS: return "Bus";
            default: return "Unknown";
        }
    }
    
    void update(float deltaTime, std::vector<Car*>& allCars, sf::Vector2f playerPos)
    {
        // Calcola distanza dal player
        distanceToPlayer = std::sqrt(std::pow(x - playerPos.x, 2) + std::pow(y - playerPos.y, 2));
        
        // Disattiva se troppo lontano
        if (distanceToPlayer > VIEW_DISTANCE)
        {
            isActive = false;
            return;
        }
        isActive = true;
        
        // Update timers
        if (aggressionTimer > 0.f) aggressionTimer -= deltaTime;
        if (honkTimer > 0.f) honkTimer -= deltaTime;
        
        // Gestione stop agli incroci con comportamento basato su aggressività
        if (stopTimer > 0.f)
        {
            stopTimer -= deltaTime;
            
            // Veicoli aggressivi aspettano meno tempo
            float aggressionReduction = properties.aggressiveness * 0.3f;
            if (aggressionReduction > 0.f && aggressionTimer <= 0.f)
            {
                stopTimer -= aggressionReduction;
                aggressionTimer = 2.0f; // Cooldown per aggressività
            }
            
            if (stopTimer <= 0.f)
            {
                // Dopo la fermata, decidi nuova direzione
                if (currentRoadPoint && currentRoadPoint->isIntersection)
                {
                    currentDirection = getDirectionChoice(currentRoadPoint, currentDirection);
                    updateRotation();
                }
                findNextTarget();
            }
            return;
        }
        
        // Controlla collisioni con comportamento personalizzato
        checkCollisions(allCars);
        
        // Regola velocità basata su situazione e personalità
        updateSpeed(deltaTime, allCars);
        
        // Movimento verso target
        if (targetRoadPoint)
        {
            moveToTarget(deltaTime);
            
            // Controlla se siamo arrivati al target
            float dx = x - targetRoadPoint->worldX;
            float dy = y - targetRoadPoint->worldY;
            float dist = std::sqrt(dx * dx + dy * dy);
            
            if (dist < 20.0f)
            {
                // Snap al target esatto
                x = targetRoadPoint->worldX;
                y = targetRoadPoint->worldY;
                currentRoadPoint = targetRoadPoint;
                
                // Se è un incrocio, fermati (tempo basato su tipo veicolo)
                if (currentRoadPoint->isIntersection)
                {
                    float baseStopTime = (properties.type == BUS) ? 1.5f : 0.8f;
                    stopTimer = baseStopTime + std::uniform_real_distribution<float>(0.f, 0.5f)(*rng);
                }
                else
                {
                    // Continua al prossimo punto
                    findNextTarget();
                }
            }
        }
        
        // Aggiorna grafica
        carShape.setPosition(sf::Vector2f(x, y));
        
        // Effetto lampeggiante per veicoli che suonano il clacson
        if (isHonking && honkTimer > 0.f)
        {
            float intensity = 0.8f + 0.2f * std::sin(honkTimer * 20.0f);
            sf::Color originalColor = properties.color;
            sf::Color flashColor = sf::Color(
                static_cast<std::uint8_t>(std::clamp(originalColor.r * intensity, 0.f, 255.f)),
                static_cast<std::uint8_t>(std::clamp(originalColor.g * intensity, 0.f, 255.f)),
                static_cast<std::uint8_t>(std::clamp(originalColor.b * intensity, 0.f, 255.f))
            );
            carShape.setFillColor(flashColor);
        }
        else
        {
            carShape.setFillColor(properties.color);
            isHonking = false;
        }
    }
    
    Direction getDirectionChoice(RoadPoint* point, Direction currentDir)
    {
        if (!point || !point->isIntersection)
            return currentDir;
        
        std::vector<Direction> validDirs;
        Direction opposite = static_cast<Direction>((currentDir + 2) % 4);
        
        // Aggiungi tutte le direzioni valide eccetto quella opposta
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
        
        // Probabilità di continuare dritto basata su tipo veicolo
        float straightProbability = 0.7f;
        if (properties.type == SPORTS_CAR) straightProbability = 0.5f; // Auto sportive più imprevedibili
        if (properties.type == BUS || properties.type == TRUCK) straightProbability = 0.9f; // Mezzi pesanti più prevedibili
        
        if (std::find(validDirs.begin(), validDirs.end(), currentDir) != validDirs.end())
        {
            if (std::uniform_real_distribution<float>(0.f, 1.f)(*rng) < straightProbability)
                return currentDir;
        }
        
        // Scegli direzione casuale
        std::uniform_int_distribution<size_t> dirDist(0, validDirs.size() - 1);
        return validDirs[dirDist(*rng)];
    }
    
    void updateSpeed(float deltaTime, std::vector<Car*>& /*allCars*/)
    {
        float targetSpeed = baseSpeed;
        
        if (isStopping)
        {
            // Velocità di frenata basata su tipo veicolo
            float brakeRate = (properties.type == TRUCK || properties.type == BUS) ? 200.0f : 400.0f;
            currentSpeed = std::max(0.0f, currentSpeed - brakeRate * deltaTime);
            
            // Veicoli aggressivi suonano il clacson quando bloccati
            if (properties.aggressiveness > 0.5f && currentSpeed < 20.0f && honkTimer <= 0.f)
            {
                honkTimer = 0.5f;
                isHonking = true;
            }
        }
        else
        {
            // Accelerazione basata su tipo veicolo
            float accelRate = 300.0f;
            if (properties.type == SPORTS_CAR) accelRate = 500.0f;
            if (properties.type == TRUCK || properties.type == BUS) accelRate = 150.0f;
            
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
        
        // Trova il prossimo punto nella direzione corrente
        targetRoadPoint = roadNetwork->getNextRoadPoint(
            currentRoadPoint->gridX, 
            currentRoadPoint->gridY, 
            currentDirection
        );
        
        // Se non troviamo un target, prova direzioni alternative
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
                    currentDirection
                );
            }
            else
            {
                // Se la scelta è di rimanere, prova comunque a trovare next
                targetRoadPoint = roadNetwork->getNextRoadPoint(
                    currentRoadPoint->gridX, 
                    currentRoadPoint->gridY, 
                    currentDirection
                );
            }
        }
    }
    
    void checkCollisions(std::vector<Car*>& allCars)
    {
        isStopping = false;
        float checkDistance = 70.0f;
        
        // Mezzi pesanti hanno distanza di sicurezza maggiore
        if (properties.type == TRUCK || properties.type == BUS)
            checkDistance = 90.0f;
        // Auto sportive hanno distanza minore (più aggressive)
        else if (properties.type == SPORTS_CAR && properties.aggressiveness > 0.6f)
            checkDistance = 50.0f;
        
        for (Car* other : allCars)
        {
            if (other == this || !other->isActive)
                continue;
            
            float dx = other->x - x;
            float dy = other->y - y;
            float dist = std::sqrt(dx * dx + dy * dy);
            
            if (dist < checkDistance)
            {
                // Verifica se è davanti nella stessa direzione
                bool isAhead = false;
                float laneWidth = (properties.type == BUS || properties.type == TRUCK) ? 40.0f : 30.0f;
                
                switch (currentDirection)
                {
                    case NORTH: isAhead = (dy < -10 && std::abs(dx) < laneWidth); break;
                    case SOUTH: isAhead = (dy > 10 && std::abs(dx) < laneWidth); break;
                    case EAST: isAhead = (dx > 10 && std::abs(dy) < laneWidth); break;
                    case WEST: isAhead = (dx < -10 && std::abs(dy) < laneWidth); break;
                    default: break;
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
        
        RoadPoint* nearest = roadNetwork->getNearestRoadPoint(x, y);
        if (nearest)
        {
            x = nearest->worldX;
            y = nearest->worldY;
            currentRoadPoint = nearest;
        }
    }
    
    void updateRotation()
    {
        switch (currentDirection)
        {
            case NORTH: carShape.setRotation(sf::degrees(270)); break;
            case EAST: carShape.setRotation(sf::degrees(0)); break;
            case SOUTH: carShape.setRotation(sf::degrees(90)); break;
            case WEST: carShape.setRotation(sf::degrees(180)); break;
            default: break;
        }
    }
    
    void draw(sf::RenderWindow& window)
    {
        if (isActive)
        {
            window.draw(carShape);
            
            // Debug: disegna linea al target
            // (commentata per performance, decommenta se ti serve)
            // if (targetRoadPoint)
            // {
            //     sf::Vertex line[2];
            //     line[0].position = sf::Vector2f(x, y);
            //     line[0].color = sf::Color::Red;
            //     line[1].position = sf::Vector2f(targetRoadPoint->worldX, targetRoadPoint->worldY);
            //     line[1].color = sf::Color::Red;
            //     window.draw(line, 2, sf::Lines);
            // }
        }
    }
}; // fine struct Car

// Car Manager migliorato
class CarManager
{
private:
    std::vector<std::unique_ptr<Car>> cars;
    std::mt19937 rng;
    RoadNetwork roadNetwork;
    float spawnTimer;
    
public:
    CarManager() : rng(std::random_device{}()), spawnTimer(0.f) {}
    
    void initialize(MapGenerator& gen)
    {
        roadNetwork.generateNetwork(gen);
        
        // Spawn iniziale molto più alto - riempi la mappa!
        for (int i = 0; i < 15; i++)
        {
            spawnCar(gen);
        }
        
        std::cout << "CarManager initialized with " << cars.size() << " initial cars" << std::endl;
    }
    
    void update(float deltaTime, MapGenerator& gen, sf::Vector2f playerPos)
    {
        // Update auto esistenti
        std::vector<Car*> carPtrs;
        carPtrs.reserve(cars.size());
        for (auto& car : cars)
        {
            carPtrs.push_back(car.get());
        }
        
        for (auto& car : cars)
        {
            car->update(deltaTime, carPtrs, playerPos);
        }
        
        // Spawn molto più frequente per mantenere traffico intenso
        spawnTimer += deltaTime;
        if (spawnTimer > 2.0f) // Spawn ogni 2 secondi invece di 5
        {
            spawnTimer = 0.f;
            
            int activeCars = 0;
            for (auto& car : cars)
            {
                if (car->isActive)
                    activeCars++;
            }
            
            // Spawn multiplo per riempire velocemente
            if (activeCars < MAX_CARS)
            {
                int spawnCount = std::min(3, MAX_CARS - activeCars); // Spawn fino a 3 macchine alla volta
                for (int i = 0; i < spawnCount; i++)
                {
                    spawnCar(gen);
                }
            }
        }
        
        // Rimuovi auto troppo lontane
        cars.erase(std::remove_if(cars.begin(), cars.end(),
            [playerPos](const std::unique_ptr<Car>& car)
            {
                return car->distanceToPlayer > VIEW_DISTANCE * 3.0f; // Rimuovi solo se molto lontano
            }), cars.end());
    }
    
    void spawnCar(MapGenerator& gen)
    {
        const auto& roadPoints = roadNetwork.getRoadPoints();
        const auto& intersections = roadNetwork.getIntersections();
        
        if (roadPoints.empty())
        {
            std::cout << "ERROR: No road points available for spawning!" << std::endl;
            return;
        }
        
        // Spawn sia su strade normali che su incroci per maggiore varietà
        std::vector<RoadPoint*> allSpawnPoints;
        allSpawnPoints.reserve(roadPoints.size() + intersections.size());
        
        // Aggiungi tutte le strade normali
        for (auto& road : roadPoints)
        {
            if (!road->isIntersection)
                allSpawnPoints.push_back(road.get());
        }
        
        // Aggiungi alcuni incroci (20% dei casi)
        std::uniform_real_distribution<float> intersectionChance(0.f, 1.f);
        if (intersectionChance(rng) < 0.2f)
        {
            for (auto& intersection : intersections)
            {
                allSpawnPoints.push_back(intersection.get());
            }
        }
        
        if (allSpawnPoints.empty())
        {
            std::cout << "No valid spawn points found!" << std::endl;
            return;
        }
        
        // Prova a spawnaare evitando aree troppo affollate
        for (int attempts = 0; attempts < 50; attempts++)
        {
            std::uniform_int_distribution<size_t> pointDist(0, allSpawnPoints.size() - 1);
            RoadPoint* spawnPoint = allSpawnPoints[pointDist(rng)];
            
            if (!spawnPoint) continue;
            
            // Controlla se l'area è già troppo affollata
            bool tooManyNearby = false;
            int nearbyCount = 0;
            for (auto& car : cars)
            {
                if (!car->isActive) continue;
                
                float dx = car->x - spawnPoint->worldX;
                float dy = car->y - spawnPoint->worldY;
                float dist = std::sqrt(dx * dx + dy * dy);
                
                if (dist < 200.0f) // Area di controllo
                {
                    nearbyCount++;
                    if (nearbyCount >= 4) // Max 4 macchine in area 200px
                    {
                        tooManyNearby = true;
                        break;
                    }
                }
            }
            
            if (tooManyNearby) continue;
            
            // Determina direzione basata sul tipo di strada
            Direction dir = NONE;
            
            if (spawnPoint->isIntersection)
            {
                // Per gli incroci, scegli una direzione valida casuale
                std::vector<Direction> validDirs;
                if (spawnPoint->canGoNorth) validDirs.push_back(NORTH);
                if (spawnPoint->canGoSouth) validDirs.push_back(SOUTH);
                if (spawnPoint->canGoEast) validDirs.push_back(EAST);
                if (spawnPoint->canGoWest) validDirs.push_back(WEST);
                
                if (!validDirs.empty())
                {
                    std::uniform_int_distribution<size_t> dirDist(0, validDirs.size() - 1);
                    dir = validDirs[dirDist(rng)];
                }
            }
            else
            {
                // Per le strade normali
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
                    &rng, &roadNetwork, &gen
                );
                cars.push_back(std::move(newCar));
                return;
            }
        }
        
        // Fallback: spawn semplice se il metodo intelligente fallisce
        for (int attempts = 0; attempts < 20; attempts++)
        {
            if (roadPoints.empty()) break;
            
            std::uniform_int_distribution<size_t> roadDist(0, roadPoints.size() - 1);
            RoadPoint* spawnPoint = roadPoints[roadDist(rng)].get();
            
            if (!spawnPoint || spawnPoint->isIntersection) continue;
            
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
                    &rng, &roadNetwork, &gen
                );
                cars.push_back(std::move(newCar));
                return;
            }
        }
    }
    
    void draw(sf::RenderWindow& window)
    {
        for (auto& car : cars)
        {
            car->draw(window);
        }
    }
    
    void drawNetwork(sf::RenderWindow& window)
    {
        roadNetwork.drawNetwork(window);
    }
    
    size_t getActiveCarCount() const
    {
        size_t count = 0;
        for (auto& car : cars)
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

// Resto del codice (loadTextures, drawMap, Player, main)
void loadTextures()
{
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
    if (!hasPlayerIdle) std::cout << "Warning: player idle texture not loaded\n";
    hasPlayerWalk = PlayerTextureWalk.loadFromFile("./assets/player_moving.png");
    if (!hasPlayerWalk) std::cout << "Warning: player walk texture not loaded\n";
    hasPlayerShoot = PlayerTextureShoot.loadFromFile("./assets/player_shooting.png");
    if (!hasPlayerShoot) std::cout << "Warning: player shoot texture not loaded\n";
    hasCarTexture1 = CarTexture1.loadFromFile("./assets/car_1.png");
    if (!hasCarTexture1) std::cout << "Warning: car texture not loaded\n";
}

void drawMap(sf::RenderWindow& window, MapGenerator& gen)
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
                if (hasStreetTexture) rect.setTexture(&streetTexture);
                else rect.setFillColor(sf::Color(80, 80, 80));
                break;
            case SIDEWALK:
                if (hasSidewalkTexture) rect.setTexture(&sidewalkTexture);
                else rect.setFillColor(sf::Color(180, 180, 180));
                break;
            case BUILDING:
                if (hasBuildingTexture) rect.setTexture(&buildingTexture);
                else rect.setFillColor(sf::Color(100, 100, 150));
                break;
            case PARK:
                if (hasParkTexture) rect.setTexture(&parkTexture);
                else rect.setFillColor(sf::Color(50, 150, 50));
                break;
            case PARKING:
                if (hasParkingTexture) rect.setTexture(&parkingTexture);
                else rect.setFillColor(sf::Color(150, 150, 100));
                break;
            default:
                continue;
            }

            window.draw(rect);
        }
    }
}

// Player
struct Player
{
    sf::CircleShape shape;
    float x, y;
    float speed;
    bool isMoving;
    float rotation;
    
    Player(float startX, float startY) : x(startX), y(startY), speed(300.0f), isMoving(false), rotation(0.0f)
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
    
    void update(float deltaTime, MapGenerator& gen)
    {
        float dx = 0.f, dy = 0.f;
        bool wasMoving = isMoving;
        isMoving = false;
        
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W))
        {
            dy = -speed * deltaTime;
            rotation = 270.f;
            isMoving = true;
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S))
        {
            dy = speed * deltaTime;
            rotation = 90.f;
            isMoving = true;
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A))
        {
            dx = -speed * deltaTime;
            rotation = 180.f;
            isMoving = true;
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D))
        {
            dx = speed * deltaTime;
            rotation = 0.f;
            isMoving = true;
        }
        
        // Controlla collisioni prima di muoversi (evita edifici)
        float newX = x + dx;
        float newY = y + dy;
        
        int gridX = static_cast<int>(newX / TILE_SIZE);
        int gridY = static_cast<int>(newY / TILE_SIZE);
        
        if (gen.getCell(gridX, gridY) != BUILDING)
        {
            x = newX;
            y = newY;
        }
        
        // Aggiorna texture basata sul movimento
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
    
    void draw(sf::RenderWindow& window)
    {
        window.draw(shape);
    }
};

// Funzione main
int main()
{
    sf::RenderWindow window(sf::VideoMode({1200, 800}), "GTA-Style Traffic System (SFML 3)");
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
            }
        }
        
        // Update
        player.update(deltaTime, gen);
        carManager.update(deltaTime, gen, player.getPosition());
        
        // Camera segue il player
        camera.setCenter(player.getPosition());
        window.setView(camera);
        
        // Draw
        window.clear(sf::Color(40, 40, 40));
        
        drawMap(window, gen);
        
        if (showDebug)
        {
            carManager.drawNetwork(window);
        }
        
        carManager.draw(window);
        player.draw(window);
        
        // UI Info (usa view di default)
        sf::View uiView = window.getDefaultView();
        window.setView(uiView);
        
        
        window.display();
    }
    
    return 0;
}
