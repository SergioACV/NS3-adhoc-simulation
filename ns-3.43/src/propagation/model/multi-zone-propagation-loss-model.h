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

  /**
   * \brief Agrega una zona circular al modelo.
   * \param centerX Coordenada X del centro.
   * \param centerY Coordenada Y del centro.
   * \param radius Radio del círculo (m).
   * \param baseLossDb Pérdida base en dB.
   * \param logCoeff Coeficiente logarítmico.
   */
  void AddCircularZone(double centerX, double centerY, double radius,
                       double baseLossDb, double logCoeff);

  /**
   * \brief Define los parámetros por defecto (usados fuera de los círculos).
   * \param baseLossDb Pérdida base en dB.
   * \param logCoeff Coeficiente logarítmico.
   */
  void SetDefaultZone(double baseLossDb, double logCoeff);

  /**
   * \brief Activa o desactiva el efecto de lluvia sobre las zonas de propagación.
   *
   * Este método ajusta dinámicamente las pérdidas de propagación para simular
   * condiciones climáticas cambiantes (por ejemplo, lluvia o humedad).
   *
   * Cuando el efecto está activo, los valores de pérdida base (`baseLossDb`)
   * y coeficiente logarítmico (`logCoeff`) de cada zona aumentan,
   * simulando la atenuación adicional causada por gotas de agua en el aire.
   *
   * \param active Si es true, activa el efecto de lluvia; si es false, lo desactiva
   * y restaura los valores originales.
   */
  void SetRainEffect(bool active);


protected:
  /**
   * \brief Cálculo principal de potencia recibida (ns-3 internamente lo llama).
   */
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
  };

  std::vector<Zone> m_zones;
  double m_defaultBaseLossDb;
  double m_defaultLogCoeff;
};

} // namespace ns3

#endif /* MULTI_ZONE_PROPAGATION_LOSS_MODEL_H */
