#include "client_sendmessagediagnosticswidget.h"

#include <QCoreApplication>
#include <QLabel>
#include <QComboBox>
#include <QPlainTextEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <gui/widgets/core_addresswidget.h>

mbClientSendMessageDiagnosticsWidget::Strings::Strings() :
    subfunction(QStringLiteral("subfunc")),
    format     (QStringLiteral("format")),
    request    (QStringLiteral("request")),
    response   (QStringLiteral("response"))
{
}

const mbClientSendMessageDiagnosticsWidget::Strings &mbClientSendMessageDiagnosticsWidget::Strings::instance()
{
    static Strings s;
    return s;
}

mbClientSendMessageDiagnosticsWidget::mbClientSendMessageDiagnosticsWidget(mbClientSendMessageUi *ui, QWidget *parent) :
    mbClientSendMessageWidget(MBF_DIAGNOSTICS, ui, parent)
{
    // subfunctions
    m_cmbSubfunction = new QComboBox(this);
    m_cmbSubfunction->setObjectName(QString::fromUtf8("cmbDiagnSubfunction"));
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
    connect(m_cmbSubfunction, SIGNAL(currentIndexChanged(int)), this, SLOT(setCurrentDiagnSubfuncIndex(int)));
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
    m_txtDataRequest->setMinimumSize(QSize(0, 100));
    m_txtDataRequest->setUndoRedoEnabled(true);
    m_txtDataRequest->setReadOnly(false);

    // request data
    m_txtDataResponse = new QPlainTextEdit(this);
    m_txtDataResponse->setMinimumSize(QSize(0, 100));
    m_txtDataResponse->setUndoRedoEnabled(true);
    m_txtDataResponse->setReadOnly(true);

    // Labels
    auto lblSubfunction = new QLabel(this);
    lblSubfunction->setText(QCoreApplication::translate("mbClientSendMessageUi", "Subfunction", nullptr));

    auto lblFormat = new QLabel(this);
    lblFormat->setText(QCoreApplication::translate("mbClientSendMessageUi", "Format", nullptr));

    auto lblRequest = new QLabel(this);
    lblRequest->setText(QCoreApplication::translate("mbClientSendMessageUi", "Request", nullptr));

    auto lblResponse = new QLabel(this);
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
    verticalLayout->addLayout(horizontalLayout);
    verticalLayout->addWidget(lblRequest);
    verticalLayout->addWidget(m_txtDataRequest);
    verticalLayout->addWidget(lblResponse);
    verticalLayout->addWidget(m_txtDataResponse);
    verticalLayout->setStretch(1, 1);
    this->setLayout(verticalLayout);

    connect(m_cmbFormat, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &mbClientSendMessageDiagnosticsWidget::updateResponseData);
    updateResponseData();
}

MBSETTINGS mbClientSendMessageDiagnosticsWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();

    MBSETTINGS m;
    m[m_prefix+s.subfunction] = getSubfunction();
    m[m_prefix+s.format     ] = m_cmbFormat     ->currentText();
    m[m_prefix+s.request    ] = m_txtDataRequest ->toPlainText();
    m[m_prefix+s.response   ] = m_responseData;

    return m;
}

void mbClientSendMessageDiagnosticsWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    it = m.find(m_prefix+s.subfunction); if (it != end) setSubfunction                  (it.value().toInt   ());
    it = m.find(m_prefix+s.format     ); if (it != end) m_cmbFormat     ->setCurrentText(it.value().toString());
    it = m.find(m_prefix+s.request    ); if (it != end) m_txtDataRequest->setPlainText  (it.value().toString());
    it = m.find(m_prefix+s.response   ); if (it != end) m_responseData = it.value().toByteArray();
    updateResponseData();
}

void mbClientSendMessageDiagnosticsWidget::fillParams(mbClientMessageParams &params) const
{
    params.setSubfunction(getSubfunction());
    params.setFormat(format());
    params.setData(m_txtDataRequest->toPlainText());
    if (getSubfunction() == MBF_DIAGNOSTICS_RETURN_QUERY_DATA)
    {
        auto b = m_conv->toByteArray(params);
        params.setData(b);
        params.setCount(b.count());
    }
    else
        params.setCount(2); // Note: buff size
}

void mbClientSendMessageDiagnosticsWidget::setParams(mbClientMessageParams &params)
{
    params.setSubfunction(getSubfunction());
    params.setFormat(format());
    m_responseData = m_conv->toByteArray(params);
    updateResponseData();
}

mb::Format mbClientSendMessageDiagnosticsWidget::format() const
{
    return mb::enumFormatValueByIndex(m_cmbFormat->currentIndex());
}

uint16_t mbClientSendMessageDiagnosticsWidget::getSubfunction() const
{
    return m_diagnSubfuncNums.value(m_cmbSubfunction->currentIndex());
}

void mbClientSendMessageDiagnosticsWidget::setSubfunction(uint16_t subfunc)
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

void mbClientSendMessageDiagnosticsWidget::setCurrentDiagnSubfuncIndex(int funcIndex)
{
    uint8_t funcNum = m_diagnSubfuncNums.value(funcIndex);
    setSubfunction(funcNum);
}

void mbClientSendMessageDiagnosticsWidget::updateResponseData()
{
    if (m_responseData.length() == 0)
    {
        m_txtDataResponse->setPlainText(QString());
        return;
    }
    mbClientMessageParams params;
    params.setFunction(function());
    params.setFormat(format());
    params.setCount(m_responseData.length()*8);
    params.setData(m_responseData);
    QString s = m_conv->toVariant(params).toString();
    m_txtDataResponse->setPlainText(s);
}
