#include "multi-zone-propagation-loss-model-helper.h"
#include "ns3/log.h"
#include "ns3/multi-zone-propagation-loss-model.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("MultiZonePropagationLossModelHelper");

MultiZonePropagationLossModelHelper::MultiZonePropagationLossModelHelper()
  : m_defaultBaseLossDb(100.0),
    m_defaultLogCoeff(30.0),
    m_hasDefaultZone(false)
{
  m_factory.SetTypeId("ns3::MultiZonePropagationLossModel");
}

void
MultiZonePropagationLossModelHelper::AddZone(double centerX, double centerY, double radius, double baseLossDb, double logCoeff)
{
  ZoneConfig z = {centerX, centerY, radius, baseLossDb, logCoeff};
  m_zones.push_back(z);
}

void
MultiZonePropagationLossModelHelper::SetDefaultZone(double baseLossDb, double logCoeff)
{
  m_defaultBaseLossDb = baseLossDb;
  m_defaultLogCoeff = logCoeff;
  m_hasDefaultZone = true;
}

Ptr<PropagationLossModel>
MultiZonePropagationLossModelHelper::Create()
{
  // ✅ Guardamos referencia persistente al modelo
  m_model = m_factory.Create<MultiZonePropagationLossModel>();

  for (const auto &zone : m_zones)
    {
      m_model->AddCircularZone(zone.centerX, zone.centerY, zone.radius, zone.baseLossDb, zone.logCoeff);
    }

  if (m_hasDefaultZone)
    {
      m_model->SetDefaultZone(m_defaultBaseLossDb, m_defaultLogCoeff);
    }

  return m_model;
}

void
MultiZonePropagationLossModelHelper::SetRainEffect(bool active)
{
  if (m_model)
    {
      m_model->SetRainEffect(active);
      NS_LOG_INFO("Rain effect " << (active ? "ACTIVATED" : "DEACTIVATED"));
    }
  else
    {
      NS_LOG_WARN("SetRainEffect() llamado antes de Create(). No hay modelo activo.");
    }
}

} // namespace ns3
