#ifndef MULTI_ZONE_PROPAGATION_LOSS_MODEL_H
#define MULTI_ZONE_PROPAGATION_LOSS_MODEL_H

#include "ns3/propagation-loss-model.h"
#include "ns3/vector.h"
#include "ns3/object.h"
#include "ns3/nstime.h"
#include <vector>

namespace ns3 {

/**
 * \ingroup propagation
 * \brief Modelo de pérdida de propagación basado en zonas circulares.
 *
 * Este modelo permite definir varias zonas circulares (x, y, radio),
 * cada una con parámetros específicos de pérdida base (dB) y coeficiente
 * logarítmico. Si un nodo transmisor se encuentra dentro de una zona,
 * se aplican sus parámetros; de lo contrario, se usa la zona por defecto.
 *
 * Fórmula:
 * \f[
 *  L(d) = L_0 + n \cdot \log_{10}(d + 1)
 * \f]
 * donde:
 *  - L_0 = baseLossDb
 *  - n = logCoeff
 */
class MultiZonePropagationLossModel : public PropagationLossModel
{
public:
  static TypeId GetTypeId(void);
  MultiZonePropagationLossModel();

  void AddCircularZone(double centerX, double centerY, double radius,
                       double baseLossDb, double logCoeff);

  void SetDefaultZone(double baseLossDb, double logCoeff);

  void SetRainEffect(bool active);

  /**
   * \brief Calcula la pérdida entre dos posiciones sin necesidad de nodos.
   *
   * Este método permite consultar la pérdida de propagación (dB) entre
   * dos vectores (posiciones) y se puede usar en lógicas como
   * PeriodicProximityCheck para decidir si un paquete podría llegar.
   *
   * \param tx Posición del transmisor.
   * \param rx Posición del receptor.
   * \return Pérdida de propagación en dB.
   */
  double GetLoss(const Vector &tx, const Vector &rx) const;

protected:
  virtual double DoCalcRxPower(double txPowerDbm,
                               Ptr<MobilityModel> a,
                               Ptr<MobilityModel> b) const override;

  virtual int64_t DoAssignStreams(int64_t stream) override { return 0; }

private:
  struct Zone
  {
    double centerX;
    double centerY;
    double radius;
    double baseLossDb;
    double logCoeff;

    bool Contains(const Vector &v) const {
        double dx = v.x - centerX;
        double dy = v.y - centerY;
        return (dx*dx + dy*dy) <= radius*radius;
    }

    double GetPathLoss(double distance) const {
        return baseLossDb + logCoeff * std::log10(distance + 1.0);
    }
  };

  std::vector<Zone> m_zones;
  double m_defaultBaseLossDb;
  double m_defaultLogCoeff;
};

} // namespace ns3

#endif /* MULTI_ZONE_PROPAGATION_LOSS_MODEL_H */
