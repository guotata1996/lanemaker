#pragma once

#include <map>
#include <string>
#include <vector>

// TODO: template? for type

namespace RoadRunnerTest{class Validation; }

class IDGenerator
{
    friend class RoadRunnerTest::Validation;
public:
    IDGenerator(IDGenerator&) = delete;
    IDGenerator& operator=(const IDGenerator&) = delete;
    static IDGenerator* ForJunction();
    static IDGenerator* ForRoad();
    static IDGenerator* ForFace(); // spatial
    static IDGenerator* ForGraphics(bool temporary); // graphics
    static IDGenerator* ForVehicle();
    static void Reset();

    size_t size() const;
    std::string GenerateID(void* object); // Added to changeList
    void NotifyChange(const std::string&);  // Added to changeList
    bool FreeID(const std::string&);  // Added to changeList
    void TakeID(std::string id, void* object); // Maintain ID from serialized
    void* GetByID(std::string id);

    inline std::map<size_t, void*> PeekChanges() { return changeList; }
    std::map<size_t, void*> ConsumeChanges();
    void ClearChangeList();
    
private:
    IDGenerator(std::string aType);
    void reset();

    static IDGenerator* _junction;
    static IDGenerator* _road;
    static IDGenerator* _face;
    static IDGenerator* _vehicle;
    static IDGenerator* _graphics;
    static IDGenerator* _graphics_temp;

    std::string type;
    std::vector<bool> assigned;
    std::map<size_t, void*> assignTo;

    std::map<size_t, void*> changeList; // Changes since last time ConsumeChanges() was called
};

