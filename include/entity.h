#pragma once
#include "factory.h"
#include "component.h"
#include "dispatcher.h"
#include "entity_event.h"

namespace spiritsaway::entity_component_event
{
	class base_entity: public poly_hash_factory<base_entity, shr_ptr_t, std::string>
	{
	public:
		base_entity(Key){};
		using Component = base_component<base_entity, const std::string&>;
	protected:
		dispatcher<entity_events, std::string> _dispatcher;
		void InitComponentVec(std::uint8_t sz)
		{
			if(!components.empty())
			{
				return;  
			}
			components = std::vector<Component*>(sz, nullptr);
		}
	private:
		std::vector<Component*> components;
	public:
		template<typename C>
		C* GetComponent()
		{
			auto cur_hash_id = base_type_hash<Component>::template hash<C>();
			if(cur_hash_id >= components.size())
			{
				return {};
			}
			auto cur_val = components[cur_hash_id];
			if(!cur_val)
			{
				return {};
			}
			return dynamic_cast<C*>(cur_val);
		}
		template <typename C>
		C* AddComponent(const std::string& arg)
		{
			auto cur_hash_id = base_type_hash<Component>::template hash<C>();
			if(cur_hash_id >= components.size())
			{
				return {};
			}
			auto cur_val = components[cur_hash_id];
			if(cur_val)
			{
				delete cur_val;
			}
			auto new_val = Component::make<C>(this, arg);
			components[cur_hash_id] = static_cast<Component*>(new_val);
			new_val->SetOwner(this);
			return new_val;
		}

		template <typename C>
		bool RemoveComponent()
		{
			auto cur_hash_id = base_type_hash<Component>::template hash<C>();
			if(cur_hash_id >= components.size())
			{
				return false;
			}
			auto cur_val = components[cur_hash_id];
			if(!cur_val)
			{
				return false;

			}
			C* real_val = dynamic_cast<C*>(cur_val);
			real_val->Destroy();
			delete real_val;
			components[cur_hash_id] = nullptr;
			return true;
		}
		template <typename... Args>
		auto DispatchEvent(entity_events event, const Args&...  args)
		{
			_dispatcher.dispatch(event, args...);
		}
		template<typename... Args>
		void DispatchEvent(const std::string& event, const Args&... args)
		{
			_dispatcher.dispatch(event, args...);
		}
		template <typename V>
		listen_handler<entity_events>  AddEventListener(entity_events cur_event, std::function<void(const entity_events&, const V&)> cur_callback)
		{
			return _dispatcher.add_listener<entity_events, V>(cur_event, cur_callback);
		}
		template <typename V>
		listen_handler<std::string>  AddEventListener(const std::string& cur_event, std::function<void(const std::string&, const V&)> cur_callback)
		{
			return _dispatcher.add_listener<std::string, V>(cur_event, cur_callback);
		}
		bool RemoveEventListener(listen_handler<entity_events> cur_handler)
		{
			return _dispatcher.remove_listener(cur_handler);
		}
		bool RemoveEventListener(listen_handler<std::string> cur_handler)
		{
			return _dispatcher.remove_listener(cur_handler);
		}
	protected:
		void ClearListeners()
		{
			_dispatcher.clear();
		}
		void ClearComponents()
		{
			for(auto& one_component: components)
			{
				if(!one_component)
				{
					delete one_component;
					one_component = nullptr;
				}
			}
			components.clear();
		}
	public:
		virtual void Destroy()
		{
			ClearListeners();
			ClearComponents();
		}
		~base_entity()
		{
			Destroy();
		}
	};
}