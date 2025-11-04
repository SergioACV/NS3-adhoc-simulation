#include "ns3/mobility-model.h"
#include "ns3/vector.h"
#include "ns3/log.h"
#include "multi-zone-propagation-loss-model.h"
#include <cmath>



namespace ns3 {

NS_LOG_COMPONENT_DEFINE("MultiZonePropagationLossModel");
NS_OBJECT_ENSURE_REGISTERED(MultiZonePropagationLossModel);

TypeId
MultiZonePropagationLossModel::GetTypeId(void)
{
  static TypeId tid = TypeId("ns3::MultiZonePropagationLossModel")
      .SetParent<PropagationLossModel>()
      .SetGroupName("Propagation")
      .AddConstructor<MultiZonePropagationLossModel>();
  return tid;
}

MultiZonePropagationLossModel::MultiZonePropagationLossModel()
  : m_defaultBaseLossDb(70.0), m_defaultLogCoeff(25.0)
{
}

void
MultiZonePropagationLossModel::AddCircularZone(double centerX, double centerY,
                                               double radius,
                                               double baseLossDb, double logCoeff)
{
  Zone z = {centerX, centerY, radius, baseLossDb, logCoeff};
  m_zones.push_back(z);
}

void
MultiZonePropagationLossModel::SetDefaultZone(double baseLossDb, double logCoeff)
{
  m_defaultBaseLossDb = baseLossDb;
  m_defaultLogCoeff = logCoeff;
}

double
MultiZonePropagationLossModel::DoCalcRxPower(double txPowerDbm,
                                             Ptr<MobilityModel> a,
                                             Ptr<MobilityModel> b) const
{
  Vector posA = a->GetPosition();
  Vector posB = b->GetPosition();
  double distance = CalculateDistance(posA, posB);

  double x = posA.x;
  double y = posA.y;

  // Buscar si el transmisor está dentro de alguna zona circular
  for (const auto &zone : m_zones)
    {
      double dx = x - zone.centerX;
      double dy = y - zone.centerY;
      double distToCenter = std::sqrt(dx * dx + dy * dy);

      if (distToCenter <= zone.radius)
        {
          double pathLossDb = zone.baseLossDb + zone.logCoeff * std::log10(distance + 1.0);
          return txPowerDbm - pathLossDb;
        }
    }

  // Si no está en ninguna zona → usa los parámetros por defecto
  double pathLossDb = m_defaultBaseLossDb + m_defaultLogCoeff * std::log10(distance + 1.0);
  return txPowerDbm - pathLossDb;
}

void
MultiZonePropagationLossModel::SetRainEffect(bool active)
{
  for (auto &zone : m_zones)
    {
      if (active)
        {
          // Aumenta las pérdidas por la lluvia
          zone.baseLossDb += 10.0;   // más pérdida base
          zone.logCoeff += 5.0;      // más dispersión
        }
      else
        {
          // Restablece valores originales si quieres mantenerlos
          zone.baseLossDb -= 10.0;
          zone.logCoeff -= 5.0;
        }
    }

  NS_LOG_INFO("Rain effect " << (active ? "ENABLED" : "DISABLED"));
}

double MultiZonePropagationLossModel::GetLoss(const Vector &tx, const Vector &rx) const
{
    // Calcular distancia euclidiana 2D
    double dist = std::sqrt((tx.x - rx.x)*(tx.x - rx.x) + (tx.y - rx.y)*(tx.y - rx.y));
    if (dist < 1.0)
        dist = 1.0; // evita log10(0)

    // Determinar zona según el transmisor (CH)
    const Zone* activeZone = nullptr;
    for (const auto &zone : m_zones)
    {
        if (zone.Contains(tx))  // << cambio clave
        {
            activeZone = &zone;
            break;
        }
    }

    // Parámetros de la zona
    double baseLoss = activeZone ? activeZone->baseLossDb : m_defaultBaseLossDb;
    double n = activeZone ? activeZone->logCoeff : m_defaultLogCoeff / 10.0;

    // Reducir pérdida a distancias muy cortas (opcional)
    if (dist < 10.0)
        baseLoss -= (10.0 - dist) * 2.0;

    // Pérdida logarítmica
    double pathLoss = baseLoss + 10.0 * n * std::log10(dist);

    return pathLoss;
}







} // namespace ns3
