#pragma once

#include <map>
#include <string>
#include <vector>
#include <memory>

namespace LTest{class Validation; }


enum class IDType
{
    Junction,
    Face,
    Road,
    Vehicle,
    Graphics,
    Graphics_Temporary,
    UI_Layover,
    Count
};

class IDGenerator
{
    friend class LTest::Validation;
public:
    IDGenerator() = default;
    IDGenerator(IDGenerator&) = delete;
    IDGenerator& operator=(const IDGenerator&) = delete;
    static std::unique_ptr<IDGenerator>& ForType(IDType);
    static void Reset();

    size_t size() const;
    std::string GenerateID(void* object); // Added to changeList
    void NotifyChange(const std::string&);  // Added to changeList
    bool FreeID(const std::string&);  // Added to changeList
    void TakeID(std::string id, void* object); // Maintain ID from serialized

    template <class T>
    T* GetByID(std::string sid)
    {
        auto id = static_cast<size_t>(std::atoi(sid.c_str()));
        if (assignTo.find(id) == assignTo.end())
        {
            return nullptr;
        }
        auto ptr = assignTo.at(id);
        if (ptr == nullptr)
        {
            return nullptr;
        }
        return static_cast<T*>(ptr);
    }

    inline std::map<size_t, void*> PeekChanges() { return changeList; }
    std::map<size_t, void*> ConsumeChanges();
    void ClearChangeList();
    
private:
    void reset();

    std::vector<bool> assigned;
    std::map<size_t, void*> assignTo;

    std::map<size_t, void*> changeList; // Changes since last time ConsumeChanges() was called
};

