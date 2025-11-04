#ifndef MULTI_ZONE_PROPAGATION_LOSS_MODEL_HELPER_H
#define MULTI_ZONE_PROPAGATION_LOSS_MODEL_HELPER_H

#include "ns3/propagation-loss-model.h"
#include "ns3/object-factory.h"
#include "ns3/ptr.h"
#include "ns3/multi-zone-propagation-loss-model.h"
#include <vector>

namespace ns3 {

/**
 * \brief Helper para configurar y controlar un MultiZonePropagationLossModel (zonas circulares).
 *
 * Este helper facilita la creación de modelos de propagación con múltiples zonas,
 * y además permite activar efectos ambientales dinámicos (como lluvia) durante la simulación.
 *
 * Ejemplo de uso:
 * \code
 * MultiZonePropagationLossModelHelper helper;
 * helper.AddZone(0.0, 0.0, 40.0, 40.0, 20.0);
 * helper.AddZone(100.0, 0.0, 40.0, 60.0, 25.0);
 * helper.AddZone(50.0, 86.6, 40.0, 80.0, 30.0);
 *
 * Ptr<PropagationLossModel> model = helper.Create();
 * channel->SetPropagationLossModel(model);
 *
 * // Activar lluvia en el tiempo t = 30s
 * Simulator::Schedule(Seconds(30.0), &MultiZonePropagationLossModelHelper::SetRainEffect, &helper, true);
 * \endcode
 */
class MultiZonePropagationLossModelHelper
{
public:
  MultiZonePropagationLossModelHelper();

  /**
   * \brief Añade una zona circular de propagación personalizada.
   * \param centerX Coordenada X del centro de la zona.
   * \param centerY Coordenada Y del centro de la zona.
   * \param radius Radio de la zona (en metros).
   * \param baseLossDb Pérdida base (en dB).
   * \param logCoeff Coeficiente logarítmico de atenuación.
   */
  void AddZone(double centerX, double centerY, double radius, double baseLossDb, double logCoeff);

  /**
   * \brief Define los parámetros por defecto para el área fuera de todas las zonas.
   * \param baseLossDb Pérdida base fuera de zonas definidas.
   * \param logCoeff Coeficiente logarítmico fuera de zonas.
   */
  void SetDefaultZone(double baseLossDb, double logCoeff);

  /**
   * \brief Crea la instancia configurada del modelo de propagación.
   * \return Puntero al modelo de propagación creado.
   */
  Ptr<PropagationLossModel> Create();

  /**
   * \brief Activa o desactiva el efecto de lluvia en el modelo.
   *
   * Ajusta dinámicamente las pérdidas (`baseLossDb` y `logCoeff`) de cada zona.
   * \param active Si es true, aumenta las pérdidas simulando lluvia; si es false, las restaura.
   */
  void SetRainEffect(bool active);

  double GetLoss(const Vector &tx, const Vector &rx) const;


private:
  ObjectFactory m_factory;

  struct ZoneConfig
  {
    double centerX;
    double centerY;
    double radius;
    double baseLossDb;
    double logCoeff;
  };

  std::vector<ZoneConfig> m_zones;
  double m_defaultBaseLossDb;
  double m_defaultLogCoeff;
  bool m_hasDefaultZone;

  // 🔹 Aquí guardamos la referencia al modelo activo
  Ptr<MultiZonePropagationLossModel> m_model;
};

} // namespace ns3

#endif /* MULTI_ZONE_PROPAGATION_LOSS_MODEL_HELPER_H */
