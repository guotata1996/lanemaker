#include "Junction.h"

namespace odr
{

JunctionLaneLink::JunctionLaneLink(int from, int to, double overlap) : from(from), to(to), overlapZone(overlap) {}

JunctionConnection::JunctionConnection(std::string id, std::string incoming_road, std::string connecting_or_linked_road, 
    ContactPoint contact_point, std::set<int> signalPhase, ContactPoint interface_provider_contact) :
    id(id), incoming_road(incoming_road), connecting_road(connecting_or_linked_road), 
    contact_point(contact_point),
    interface_provider_contact(interface_provider_contact), signalPhases(signalPhase)
{
}

JunctionConnection::JunctionConnection(std::string   id,
                                       std::string   incoming_road,
                                       std::string   connecting_or_linked_road,
                                       ContactPoint  contact_point,
                                       ContactPoint  interface_provider_contact) :
    id(id),
    incoming_road(incoming_road), connecting_road(connecting_or_linked_road), contact_point(contact_point),
    interface_provider_contact(interface_provider_contact)
{
}

JunctionPriority::JunctionPriority(std::string high, std::string low) : high(high), low(low) {}

JunctionController::JunctionController(std::string id, std::string type, std::uint32_t sequence) : id(id), type(type), sequence(sequence) {}

Junction::Junction(std::string name, std::string id, JunctionType type) : name(name), id(id), type(type) {}

} // namespace odr