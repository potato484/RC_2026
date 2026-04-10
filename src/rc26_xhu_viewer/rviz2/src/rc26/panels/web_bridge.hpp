#ifndef RC26_XHU_VIEWER__PANELS__WEB_BRIDGE_HPP_
#define RC26_XHU_VIEWER__PANELS__WEB_BRIDGE_HPP_

#include <QObject>
#include <QString>

namespace rc26_xhu_viewer
{

class WebBridge : public QObject
{
  Q_OBJECT
  Q_PROPERTY(QString lastData READ lastData NOTIFY dataChanged)

public:
  explicit WebBridge(QObject * parent = nullptr);
  QString lastData() const;
  void pushData(const QString & json);

Q_SIGNALS:
  void dataChanged();

private:
  QString last_data_;
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__PANELS__WEB_BRIDGE_HPP_
