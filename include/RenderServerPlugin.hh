/*
   Copyright (C) 2023 ArduPilot.org

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef RENDERSERVERPLUGIN_HH_
#define RENDERSERVERPLUGIN_HH_

#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/rendering/Scene.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include <gz/sim/Server.hh>
#include <gz/sim/System.hh>


namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

  class RenderServerPlugin:
    public System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    private: std::vector<common::ConnectionPtr> connections;

    private: rendering::ScenePtr scene{nullptr};

    private: EventManager *eventMgr{nullptr};

    void FindScene()
    {
      auto loadedEngNames = gz::rendering::loadedEngines();

      auto engineName = loadedEngNames[0];
      auto engine = gz::rendering::engine(engineName);

      auto scenePtr = engine->SceneByIndex(0);

      this->scene = scenePtr;
    };

    public: void OnPreRender()
    {
      if (this->scene == nullptr)
      {
        this->FindScene();
        gzdbg << "Found Scene.\n";
      }

      gzdbg << "OnPreRender.\n";
    }

    public: void OnPostRender()
    {
      gzdbg << "OnPostRender.\n";
    }

    public: void Configure(
      const Entity &,
      const std::shared_ptr<const sdf::Element> &,
      EntityComponentManager &,
      EventManager &_eventMgr
    )
    {
      gzdbg << "RenderServerPlugin: configuring.\n";

      this->connections.push_back(
        _eventMgr.Connect<events::PreRender>(
          std::bind(&RenderServerPlugin::OnPreRender, this)
        )
      );

      this->connections.push_back(
        _eventMgr.Connect<events::PostRender>(
          std::bind(&RenderServerPlugin::OnPostRender, this)
        )
      );

      this->eventMgr = &_eventMgr;
    };

    public: void PreUpdate(
      const UpdateInfo &,
      EntityComponentManager &)
    {
      // gzdbg << "RenderServerPlugin: pre-updating.\n";
      this->eventMgr->Emit<events::ForceRender>();
    };
  };

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

#endif  // RENDERSERVERPLUGIN_HH_
