#include "junction.h"
#include <sstream>

#include "id_generator.h"
#include "constants.h"

namespace RoadRunner
{
    AbstractJunction::AbstractJunction() :
        generated("", IDGenerator::ForJunction()->GenerateID(this), odr::JunctionType::Common)
    {
        generated.name = "Junction " + generated.id;
    }

    AbstractJunction::AbstractJunction(const odr::Junction& serialized) :
        generated(serialized)
    {
        IDGenerator::ForJunction()->TakeID(ID(), this);
    }

    int AbstractJunction::Attach(ConnectionInfo conn)
    {
        if (formedFrom.find(conn) != formedFrom.end())
        {
            generationError = Junction_DuplicateConn;
            return generationError;
        }
        auto connRoad = conn.road.lock();
        double connS = conn.contact == odr::RoadLink::ContactPoint_Start ?
            0 : connRoad->Length();
        RoadRunner::CubicSplineGenerator::OverwriteSection(
            connRoad->RefLine().elevation_profile, connRoad->Length(), connS, connS, Elevation());
#ifndef G_TEST
        connRoad->GenerateOrUpdateSectionGraphicsBetween(
            std::max(connS - RoadRunner::CubicSplineGenerator::MaxTransitionLength, 0.0),
            std::min(connS + RoadRunner::CubicSplineGenerator::MaxTransitionLength, connRoad->Length()));
#endif

        std::vector<ConnectionInfo> newConnections = { conn };
        for (auto existing : formedFrom)
        {
            // if (existing.road.expired())
            //    continue;
            ConnectionInfo existingInfo{ existing.road.lock(), existing.contact, existing.skipProviderLanes };
            newConnections.push_back(existingInfo);
        }
        formedFrom.clear();
        return CreateFrom(newConnections);
    }

    void AbstractJunction::NotifyPotentialChange()
    {
        NotifyPotentialChange(ChangeInConnecting());
    }

    void AbstractJunction::NotifyPotentialChange(const ChangeInConnecting& detail)
    {
        std::shared_ptr<Road> subject;
        if (detail._type != detail.Type_Others)
        {
            subject = detail.subject.lock();
        }

        std::vector<ConnectionInfo> updatedInfoList;
        bool needReGen{ false };
        for (const auto record : formedFrom)
        {
            auto recordedRoad = record.road.lock();

            if (recordedRoad == nullptr)
            {
                needReGen = true;
            }
            else if (detail._type == detail.Type_Reverse && subject == recordedRoad)
            {
                needReGen = true;

                auto newContact = record.contact == odr::RoadLink::ContactPoint_Start ?
                    odr::RoadLink::ContactPoint_End : odr::RoadLink::ContactPoint_Start;
                auto updatedInfo = ConnectionInfo(recordedRoad, newContact, record.skipProviderLanes);
                updatedInfoList.push_back(updatedInfo);
            }
            else if (detail._type == detail.Type_DetachAtEnd_Temp && record.contact == odr::RoadLink::ContactPoint_End &&
                subject == recordedRoad)
            {
                subject->generated.successor = odr::RoadLink();
                subject->successorJunction.reset();
            }
            else
            {
                // Must recalculate profile,pos,hdg to see if update is needed
                auto updatedInfo = ConnectionInfo(recordedRoad, record.contact, record.skipProviderLanes);
                updatedInfoList.push_back(updatedInfo);
                if (updatedInfo != record)
                    needReGen = true;
            }
        }

        if (detail._type == detail.Type_DetachAtEnd_Temp)
        {
            formedFrom.clear();
            formedFrom.insert(updatedInfoList.cbegin(), updatedInfoList.cend());
        }
        else if (needReGen && updatedInfoList.size() > 1)
        {
            spdlog::trace("Junction {} regen from {} roads", ID(), updatedInfoList.size());
            formedFrom.clear();
            CreateFrom(updatedInfoList);
        }
        else if (updatedInfoList.size() == 1)
        {
            auto onlyRoad = updatedInfoList.begin()->road.lock();
            if (updatedInfoList.begin()->contact == odr::RoadLink::ContactPoint_Start)
            {
                onlyRoad->predecessorJunction.reset();
            }
            else if (updatedInfoList.begin()->contact == odr::RoadLink::ContactPoint_End)
            {
                onlyRoad->successorJunction.reset();
            }
            else
            {
                assert(false);
            }

            clearLinkage(ID(), onlyRoad->ID());
            IDGenerator::ForRoad()->NotifyChange(onlyRoad->ID());
            formedFrom.clear();
            // Junction will then be destroyed
        }
    }

    std::set<std::shared_ptr<Road>> AbstractJunction::StillConnectedRoads() const
    {
        std::set<std::shared_ptr<Road>> rtn;
        for (auto connected : formedFrom)
        {
            auto sharedPtr = connected.road.lock();
            if (sharedPtr != nullptr)
            {
                rtn.insert(sharedPtr);
            }
        }
        return rtn;
    }

    void AbstractJunction::AttachNoRegenerate(ConnectionInfo conn)
    {
        formedFrom.insert(conn);
        if (conn.contact == odr::RoadLink::ContactPoint_Start)
        {
            conn.road.lock()->predecessorJunction = shared_from_this();
        }
        else
        {
            conn.road.lock()->successorJunction = shared_from_this();
        }
    }

    void AbstractJunction::DetachNoRegenerate(std::shared_ptr<Road> road)
    {
        auto myPtr = shared_from_this();
        if (road->successorJunction == myPtr)
        {
            formedFrom.erase(RoadRunner::ConnectionInfo(road, odr::RoadLink::ContactPoint_End));
            road->successorJunction.reset();
        }

        if (road->predecessorJunction == myPtr)
        {
            formedFrom.erase(RoadRunner::ConnectionInfo(road, odr::RoadLink::ContactPoint_Start));
            road->predecessorJunction.reset();
        }
    }

    std::string AbstractJunction::Log() const
    {
        std::stringstream ss;
        ss << "Junction " << ID() << "\n";
        for (auto contact : formedFrom)
        {
            auto contactStr = contact.contact == odr::RoadLink::ContactPoint_Start ? "Start" :
                contact.contact == odr::RoadLink::ContactPoint_End ? "End" : "None";
            ss << "    " << contact.road.lock()->ID() << " connected at " << contactStr << "\n";
        }
        return ss.str();
    }

    void AbstractJunction::FillConnectionInfo(ConnectionInfo& info) const
    {
        auto record = formedFrom.find(info);
        info = *record;
    }

    double AbstractJunction::Elevation() const
    {
        const auto road = formedFrom.begin()->road.lock();
        auto s = formedFrom.begin()->contact == odr::RoadLink::ContactPoint_Start ? 0 :
            road->Length();
        return road->RefLine().elevation_profile.get(s);
    }

    Junction::Junction() : AbstractJunction() {}

    Junction::Junction(const odr::Junction& serialized) : AbstractJunction(serialized)
    {
        // Link connecting road
        for (const auto& id2Connection : generated.id_to_connection)
        {
            auto connectingRoadID = id2Connection.second.connecting_road;
            auto roadPtr = static_cast<RoadRunner::Road*>(IDGenerator::ForRoad()->GetByID(connectingRoadID));
            connectingRoads.push_back(roadPtr->shared_from_this());
        }
    }

    int Junction::CreateFrom(const std::vector<ConnectionInfo>& connected)
    {
        connectingRoads.clear();

        generationError = GenerateConnections(generated.id, connected, connectingRoads);

        std::for_each(connected.begin(), connected.end(), [this](const ConnectionInfo& info) {
            formedFrom.insert(info);
            auto roadPtr = info.road.lock();
            if (info.contact == odr::RoadLink::ContactPoint_Start)
            {
                roadPtr->predecessorJunction = shared_from_this();
            }
            else
            {
                roadPtr->successorJunction = shared_from_this();
            }
        });

        generated.id_to_connection.clear();
        int junctionConnID = 0;
        for (auto& connecting : connectingRoads)
        {
            auto incomingRoad = connecting->generated.predecessor.id;

            odr::JunctionConnection prevConn(std::to_string(junctionConnID++),
                incomingRoad, connecting->ID(),
                odr::JunctionConnection::ContactPoint_Start);

            for (odr::Lane connectinglane : connecting->generated.s_to_lanesection.rbegin()->second.get_sorted_driving_lanes(-1))
            {
                prevConn.lane_links.insert(odr::JunctionLaneLink(connectinglane.predecessor, connectinglane.id));
            }
            generated.id_to_connection.emplace(prevConn.id, prevConn);

            CubicSplineGenerator::OverwriteSection(connecting->RefLine().elevation_profile,
                connecting->Length(), 0, connecting->Length(), Elevation());
        }

#ifndef G_TEST
        for (auto& connecting : connectingRoads)
        {
            if (connecting->Length() < RoadRunner::SingleDrawMaxLength)
            {
                connecting->GenerateAllSectionGraphics();
            }
            else
            {
                spdlog::warn("Connecting road length is abnormal!");
                generationError |= Junction_ConnectionInvalidShape;
            }
        }
#endif

        IDGenerator::ForJunction()->NotifyChange(ID());

        return generationError;
    }

    AbstractJunction::~AbstractJunction()
    {
        for (const auto& connectingRoad : formedFrom)
        {
            if (!connectingRoad.road.expired())
            {
                spdlog::error("Junction gets destroyed before its connected road!");
            }
        }

        if (!ID().empty())
        {
            IDGenerator::ForJunction()->FreeID(ID());
        }
    }

    DirectJunction::DirectJunction(ConnectionInfo aInterfaceProvider) : AbstractJunction()
    {
        formedFrom.insert(aInterfaceProvider);
        generated.type = odr::JunctionType::Direct;

        interfaceDir = calcInterfaceDir(aInterfaceProvider);
    }

    DirectJunction::DirectJunction(const odr::Junction& serialized) : AbstractJunction(serialized)
    {
        generated.type = odr::JunctionType::Direct;
        auto interfaceProviderID = serialized.id_to_connection.cbegin()->second.incoming_road;
        
        auto interfaceProvider = static_cast<RoadRunner::Road*>(IDGenerator::ForRoad()->GetByID(interfaceProviderID));
        odr::RoadLink::ContactPoint interfaceContact;
        if (interfaceProvider->generated.predecessor.type == odr::RoadLink::Type_Junction &&
            interfaceProvider->generated.predecessor.id == ID())
        {
            interfaceContact = odr::RoadLink::ContactPoint_Start;
        }
        else if (interfaceProvider->generated.successor.type == odr::RoadLink::Type_Junction &&
            interfaceProvider->generated.successor.id == ID())
        {
            interfaceContact = odr::RoadLink::ContactPoint_End;
        }
        else throw;

        interfaceDir = calcInterfaceDir(ConnectionInfo(interfaceProvider->shared_from_this(), interfaceContact));
    }

    int DirectJunction::CreateFrom(const std::vector<ConnectionInfo>& connectedInfo)
    {
        formedFrom.insert(connectedInfo.begin(), connectedInfo.end());

        if (formedFrom.size() != connectedInfo.size())
        {
            spdlog::error("A road cannot appear in the same direct junction twice!");
            return JunctionError::Junction_DuplicateConn;
        }

        auto interfaceInfoOrInvalid = InterfaceProvider();
        if (!interfaceInfoOrInvalid.has_value())
        {
            return JunctionError::Junction_DirectNoProvider;
        }
        auto interfaceInfo = interfaceInfoOrInvalid.value();
        std::shared_ptr<Road> interfaceProviderRoad = interfaceInfo.road.lock();
        auto interfaceContact = interfaceInfo.contact;

        if (interfaceProviderRoad == nullptr)
        {
            // Direct junction no longer holds without interface provider
            for (auto contactInfo : connectedInfo)
            {
                auto connectedRoad = contactInfo.road.lock();
                if (contactInfo.contact == odr::RoadLink::ContactPoint_Start)
                {
                    connectedRoad->predecessorJunction.reset();
                }
                else if (contactInfo.contact == odr::RoadLink::ContactPoint_End)
                {
                    connectedRoad->successorJunction.reset();
                }
                else
                {
                    throw std::logic_error("DirectJunction::CreateFrom Invalid contact point");
                }
                clearLinkage(ID(), connectedRoad->ID());
                IDGenerator::ForRoad()->NotifyChange(connectedRoad->ID());
            }
            formedFrom.clear();
            // Junction will then be destroyed
            return JunctionError::Junction_NoError;
        }

        for (const ConnectionInfo& info: connectedInfo)
        {
            auto connectedRoad = info.road.lock();
            if (info.contact == odr::RoadLink::ContactPoint_Start)
            {
                connectedRoad->predecessorJunction = shared_from_this();
                connectedRoad->generated.predecessor = odr::RoadLink(ID(), odr::RoadLink::Type_Junction);
            }
            else
            {
                connectedRoad->successorJunction = shared_from_this();
                connectedRoad->generated.successor = odr::RoadLink(ID(), odr::RoadLink::Type_Junction);
            }
            IDGenerator::ForRoad()->NotifyChange(connectedRoad->ID());
        }

        generated.id_to_connection.clear();
        int junctionConnID = 0;
        const auto& sectionProvider = interfaceProviderRoad->generated.get_lanesection(
            interfaceContact == odr::RoadLink::ContactPoint_Start ? 0 : interfaceProviderRoad->Length());
        for (auto rampInfo: connectedInfo)
        {
            auto linkedRoad = rampInfo.road.lock();
            auto linkedContact = rampInfo.contact;
            if (linkedRoad == interfaceProviderRoad)
            {
                continue;
            }
            // Here we just let incomingRoad be interface provider; while linkedRoad be narrower ramp
            odr::JunctionConnection conn(std::to_string(junctionConnID++), interfaceProviderRoad->ID(), linkedRoad->ID(),
                linkedContact == odr::RoadLink::ContactPoint_Start ? odr::JunctionConnection::ContactPoint_Start : odr::JunctionConnection::ContactPoint_End);

            const auto& sectionLinked = linkedRoad->generated.get_lanesection(linkedContact == odr::RoadLink::ContactPoint_Start ? 0 : linkedRoad->Length());
            
            // Link lanes from Provider to Linked (split)
            if (linkedRoad->generated.rr_profile.HasSide(1) ||      // bi-dir
                linkedContact == odr::RoadLink::ContactPoint_Start) // or single-dir lane split
            {
                auto lanesOnProvider = sectionProvider.get_sorted_driving_lanes(interfaceContact == odr::RoadLink::ContactPoint_End ? -1 : 1);
                auto lanesOnLinked = sectionLinked.get_sorted_driving_lanes(linkedContact == odr::RoadLink::ContactPoint_Start ? -1 : 1);

                for (int i = 0; i != lanesOnLinked.size(); ++i)
                {
                    conn.lane_links.insert(odr::JunctionLaneLink(lanesOnProvider[i + rampInfo.skipProviderLanes].id, lanesOnLinked[i].id));
                }
            }
            
            // Link lanes from Linked to Provider (merge)
            if (linkedRoad->generated.rr_profile.HasSide(-1) ||      // bi-dir
                linkedContact == odr::RoadLink::ContactPoint_End)   // or single-dir lane merge
            {
                auto lanesOnLinked = sectionLinked.get_sorted_driving_lanes(linkedContact == odr::RoadLink::ContactPoint_End ? -1 : 1);
                auto lanesOnProvider = sectionProvider.get_sorted_driving_lanes(interfaceContact == odr::RoadLink::ContactPoint_Start ? -1 : 1);
                for (int i = 0; i != lanesOnLinked.size(); ++i)
                {
                    conn.lane_links.insert(odr::JunctionLaneLink(lanesOnProvider[i + rampInfo.skipProviderLanes].id, lanesOnLinked[i].id));
                }
            }

            generated.id_to_connection.emplace(conn.id, conn);
        }

        IDGenerator::ForJunction()->NotifyChange(ID());

        return JunctionError::Junction_NoError;
    }

    void DirectJunction::AttachNoRegenerate(ConnectionInfo conn)
    {
        auto road = conn.road.lock();
        auto interfaceProvider = static_cast<Road*>(IDGenerator::ForRoad()->GetByID(generated.id_to_connection.begin()->second.incoming_road));
        bool isInterfaceProvider = interfaceProvider->ID() == conn.road.lock()->ID();
        bool connIsSide = road->generated.rr_profile.HasSide(-1) && road->generated.rr_profile.HasSide(1);
        if (!isInterfaceProvider && 
            !connIsSide)
        {
            // Recover skipProviderLanes from laneLink info
            double sectionS;
            if (interfaceProvider->generated.successor.id == ID() && interfaceProvider->generated.successor.type == odr::RoadLink::Type_Junction)
            {
                sectionS = interfaceProvider->Length();
            }
            else if (interfaceProvider->generated.predecessor.id == ID() && interfaceProvider->generated.predecessor.type == odr::RoadLink::Type_Junction)
            {
                sectionS = 0;
            }
            else
            {
                throw;
            }
            auto touchingSection = interfaceProvider->generated.get_lanesection(sectionS);

            bool recovered = false;
            for (auto idAndConn : generated.id_to_connection)
            {
                if (idAndConn.second.connecting_road == conn.road.lock()->ID())
                {
                    int innerMostLinkedABS = 255;
                    for (auto laneLink : idAndConn.second.lane_links)
                    {
                        innerMostLinkedABS = std::min(innerMostLinkedABS, std::abs(laneLink.from));
                    }

                    int interfaceProvideSide = idAndConn.second.lane_links.begin()->from < 0 ? -1 : 1;
                    int innerMostLane = touchingSection.get_sorted_driving_lanes(interfaceProvideSide).begin()->id;
                    conn.skipProviderLanes = std::abs(innerMostLinkedABS - std::abs(innerMostLane));
                    recovered = true;
                    break;
                }
            }
            assert(recovered);
        }

        AbstractJunction::AttachNoRegenerate(conn);
    }

    odr::Vec2D DirectJunction::calcInterfaceDir(const ConnectionInfo& aInterfaceProvider)
    {
        odr::Vec2D rtn;
        auto interfaceRoad = aInterfaceProvider.road.lock();
        if (aInterfaceProvider.contact == odr::RoadLink::ContactPoint_Start)
        {
            rtn = interfaceRoad->RefLine().get_grad_xy(0);
        }
        else
        {
            rtn = interfaceRoad->RefLine().get_grad_xy(interfaceRoad->Length());
            rtn = odr::negate(rtn);
        }
        rtn = odr::normalize(rtn);
        return rtn;
    }

    std::optional<ConnectionInfo> DirectJunction::InterfaceProvider() const
    {
        std::shared_ptr<Road> interfaceProviderRoad;
        odr::RoadLink::ContactPoint interfaceContact = odr::RoadLink::ContactPoint_None;

        bool matchFound = false;
        for (const auto& connInfo : formedFrom)
        {
            auto connDir = calcInterfaceDir(connInfo);
            if (odr::dot(interfaceDir, connDir) > 0.9)
            {
                interfaceProviderRoad = connInfo.road.lock();
                interfaceContact = connInfo.contact;
                if (matchFound)
                {
                    spdlog::error("More than one road match interface direction!");
                    return std::optional<ConnectionInfo>();
                }
                matchFound = true;
            }
        }
        return ConnectionInfo(interfaceProviderRoad, interfaceContact);
    }

    std::string DirectJunction::Log() const
    {
        std::stringstream ss;
        ss << "Direct Junction " << ID() << "\n";
        auto providerOrBad = InterfaceProvider();
        if (!providerOrBad.has_value())
        {
            ss << "     Error: Invalid Interface provider!\n";
        }
        else
        {
            for (auto contact : formedFrom)
            {
                auto contactStr = contact.contact == odr::RoadLink::ContactPoint_Start ? "Start" :
                    contact.contact == odr::RoadLink::ContactPoint_End ? "End" : "None";
                auto typeStr = contact == providerOrBad.value() ? "Interface " : "Linked ";
                ss << "    " << typeStr << contact.road.lock()->ID() << " connected at " << contactStr << "\n";
            }
        }
        return ss.str();
    }
} // namespace RoadRunner