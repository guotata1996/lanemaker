#pragma once

#include <map>
#include <string>
#include <vector>

// TODO: template? for type
class IDGenerator
{
public:
    IDGenerator(IDGenerator&) = delete;
    IDGenerator& operator=(const IDGenerator&) = delete;
    static IDGenerator* ForJunction();
    static IDGenerator* ForRoad();

    size_t size() const;
    std::string GenerateID(void* object); // Added to changeList
    void NotifyChange(const std::string&);  // Added to changeList
    bool FreeID(const std::string&);  // Added to changeList
    void TakeID(std::string id, void* object); // Maintain ID from serialized
    void* GetByID(std::string id);

    std::map<size_t, void*> ConsumeChanges();
    void ClearChangeList();
    
private:
    IDGenerator(std::string aType);

    static IDGenerator* _junction;
    static IDGenerator* _road;

    std::string type;
    std::vector<bool> assigned;
    std::map<size_t, void*> assignTo;

    std::map<size_t, void*> changeList; // Changes since last time ConsumeChanges() was called
};

