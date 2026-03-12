#include "client_sendmessagediagnwidget.h"

#include <QCoreApplication>
#include <QLabel>
#include <QComboBox>
#include <QPlainTextEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <gui/widgets/core_addresswidget.h>

mbClientSendMessageDiagnWidget::Strings::Strings() :
    prefix                 (QStringLiteral("Ui.SendMessage.DiagnWidget.")),
    diagnSubfunction       (prefix+QStringLiteral("subfunc")),
    diagnFormat            (prefix+QStringLiteral("format")),
    diagnRequest           (prefix+QStringLiteral("request"))
{
}

const mbClientSendMessageDiagnWidget::Strings &mbClientSendMessageDiagnWidget::Strings::instance()
{
    static Strings s;
    return s;
}

mbClientSendMessageDiagnWidget::mbClientSendMessageDiagnWidget(mbClientSendMessageUi *ui, QWidget *parent) :
    mbClientSendMessageWidget(ui, parent)
{
    this->setObjectName(QString::fromUtf8("pgDiagn"));

    // subfunctions
    m_cmbSubfunction = new QComboBox(this);
    m_cmbSubfunction->setObjectName(QString::fromUtf8("cmbDiagnSubfunction"));
    connect(m_cmbSubfunction, SIGNAL(currentIndexChanged(int)), this, SLOT(setCurrentDiagnSubfuncIndex(int)));
    m_diagnSubfuncNums.append(MBF_DIAGNOSTICS_RETURN_QUERY_DATA                     );
    m_diagnSubfuncNums.append(MBF_DIAGNOSTICS_RESTART_COMMUNICATIONS_OPTION         );
    m_diagnSubfuncNums.append(MBF_DIAGNOSTICS_RETURN_DIAGNOSTIC_REGISTER            );
    m_diagnSubfuncNums.append(MBF_DIAGNOSTICS_CHANGE_ASCII_INPUT_DELIMITER          );
    m_diagnSubfuncNums.append(MBF_DIAGNOSTICS_FORCE_LISTEN_ONLY_MODE                );
    m_diagnSubfuncNums.append(MBF_DIAGNOSTICS_CLEAR_COUNTERS_AND_DIAGNOSTIC_REGISTER);
    m_diagnSubfuncNums.append(MBF_DIAGNOSTICS_RETURN_BUS_MESSAGE_COUNT              );
    m_diagnSubfuncNums.append(MBF_DIAGNOSTICS_RETURN_BUS_COMMUNICATION_ERROR_COUNT  );
    m_diagnSubfuncNums.append(MBF_DIAGNOSTICS_RETURN_BUS_EXCEPTION_ERROR_COUNT      );
    m_diagnSubfuncNums.append(MBF_DIAGNOSTICS_RETURN_SERVER_MESSAGE_COUNT           );
    m_diagnSubfuncNums.append(MBF_DIAGNOSTICS_RETURN_SERVER_NO_RESPONSE_COUNT       );
    m_diagnSubfuncNums.append(MBF_DIAGNOSTICS_RETURN_SERVER_NAK_COUNT               );
    m_diagnSubfuncNums.append(MBF_DIAGNOSTICS_RETURN_SERVER_BUSY_COUNT              );
    m_diagnSubfuncNums.append(MBF_DIAGNOSTICS_RETURN_BUS_CHARACTER_OVERRUN_COUNT    );
    m_diagnSubfuncNums.append(MBF_DIAGNOSTICS_CLEAR_OVERRUN_COUNTER_AND_FLAG        );
    Q_FOREACH (auto funcNum, m_diagnSubfuncNums)
    {
        m_cmbSubfunction->addItem(QString("%1 - %2")
                         .arg(funcNum, 2, 10, QChar('0'))
                         .arg(mb::ModbusDiagnSubfunctionString(funcNum))
                     );
    }
    m_cmbSubfunction->setCurrentIndex(0);

    // format
    m_cmbFormat = new QComboBox(this);
    m_cmbFormat->setObjectName(QString::fromUtf8("cmbDiagnFormat"));
    auto ls = mb::enumFormatKeyList();
    Q_FOREACH (const QString &s, ls)
    {
        m_cmbFormat->addItem(s);
    }
    m_cmbFormat->setCurrentIndex(mb::Dec16);

    // request data
    m_txtDataRequest = new QPlainTextEdit(this);
    m_txtDataRequest->setObjectName(QString::fromUtf8("txtDiagnRequest"));
    m_txtDataRequest->setMinimumSize(QSize(0, 100));
    m_txtDataRequest->setUndoRedoEnabled(true);
    m_txtDataRequest->setReadOnly(true);

    // request data
    m_txtDataResponse = new QPlainTextEdit(this);
    m_txtDataResponse->setObjectName(QString::fromUtf8("txtDiagnResponse"));
    m_txtDataResponse->setMinimumSize(QSize(0, 100));
    m_txtDataResponse->setUndoRedoEnabled(true);
    m_txtDataResponse->setReadOnly(true);

    // Labels
    auto lblSubfunction = new QLabel(this);
    lblSubfunction->setObjectName(QString::fromUtf8("lblDiagnSubfunction"));
    lblSubfunction->setText(QCoreApplication::translate("mbClientSendMessageUi", "Subfunction", nullptr));

    auto lblFormat = new QLabel(this);
    lblFormat->setObjectName(QString::fromUtf8("lblDiagnFormat"));
    lblFormat->setText(QCoreApplication::translate("mbClientSendMessageUi", "Format", nullptr));

    auto lblRequest = new QLabel(this);
    lblRequest->setObjectName(QString::fromUtf8("lblDiagnRequest"));
    lblRequest->setText(QCoreApplication::translate("mbClientSendMessageUi", "Request", nullptr));

    auto lblResponse = new QLabel(this);
    lblResponse->setObjectName(QString::fromUtf8("lblDiagnResponse"));
    lblResponse->setText(QCoreApplication::translate("mbClientSendMessageUi", "Response", nullptr));

    // Layouts
    auto horizontalLayout = new QHBoxLayout();
    horizontalLayout->setObjectName(QString::fromUtf8("horizontalDiagnLayout"));
    horizontalLayout->addWidget(lblSubfunction);
    horizontalLayout->addWidget(m_cmbSubfunction);
    horizontalLayout->addWidget(lblFormat);
    horizontalLayout->addWidget(m_cmbFormat);
    horizontalLayout->setStretch(1, 1);

    auto verticalLayout = new QVBoxLayout();
    horizontalLayout->setObjectName(QString::fromUtf8("verticalDiagnLayout"));
    horizontalLayout->addLayout(horizontalLayout);
    horizontalLayout->addWidget(lblRequest);
    horizontalLayout->addWidget(m_txtDataRequest);
    horizontalLayout->addWidget(lblResponse);
    horizontalLayout->addWidget(m_txtDataResponse);
    horizontalLayout->setStretch(1, 1);

    this->setLayout(verticalLayout);
}

MBSETTINGS mbClientSendMessageDiagnWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();

    MBSETTINGS m;
    m[s.diagnSubfunction] = getSubfunction();
    m[s.diagnFormat     ] = m_cmbFormat     ->currentText();
    m[s.diagnRequest    ] = m_txtDataRequest ->toPlainText();

    return m;
}

void mbClientSendMessageDiagnWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    it = m.find(s.diagnSubfunction); if (it != end) setSubfunction                  (it.value().toInt   ());
    it = m.find(s.diagnFormat     ); if (it != end) m_cmbFormat     ->setCurrentText(it.value().toString());
    it = m.find(s.diagnRequest    ); if (it != end) m_txtDataRequest->setPlainText  (it.value().toString());
}

QByteArray mbClientSendMessageDiagnWidget::getData() const
{

}

void mbClientSendMessageDiagnWidget::setData(const QByteArray &data)
{

}

uint16_t mbClientSendMessageDiagnWidget::getSubfunction() const
{
    return m_diagnSubfuncNums.value(m_cmbSubfunction->currentIndex());
}

void mbClientSendMessageDiagnWidget::setSubfunction(uint16_t subfunc)
{
    switch (subfunc)
    {
    case MBF_DIAGNOSTICS_RETURN_QUERY_DATA:
    case MBF_DIAGNOSTICS_RETURN_DIAGNOSTIC_REGISTER:
    case MBF_DIAGNOSTICS_CHANGE_ASCII_INPUT_DELIMITER:
        m_txtDataRequest->setEnabled(true);
        break;
    default:
        m_txtDataRequest->setEnabled(false);
        break;
    }

    int i = 0;
    Q_FOREACH (int f, m_diagnSubfuncNums)
    {
        if (f == subfunc)
        {
            m_cmbSubfunction->setCurrentIndex(i);
            break;
        }
        ++i;
    }
}

void mbClientSendMessageDiagnWidget::setCurrentDiagnSubfuncIndex(int funcIndex)
{
    uint8_t funcNum = m_diagnSubfuncNums.value(funcIndex);
    setSubfunction(funcNum);
}
