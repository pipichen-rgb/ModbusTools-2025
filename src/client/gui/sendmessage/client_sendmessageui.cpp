/*
    Modbus Tools

    Created: 2023
    Author: Serhii Marchuk, https://github.com/serhmarch

    Copyright (C) 2023  Serhii Marchuk

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
#include "client_sendmessageui.h"
#include "ui_client_sendmessageui.h"

#include <QTextStream>

#include <client.h>

#include <project/client_project.h>
#include <project/client_port.h>
#include <project/client_device.h>

#include <runtime/client_runmessage.h>

#include <gui/widgets/core_addresswidget.h>

#include <gui/client_ui.h>
#include <gui/dialogs/client_dialogs.h>

#include "client_sendmessagefilerecordmodel.h"
#include "client_sendmessagelistmodel.h"

mbClientSendMessageUi::Strings::Strings() :
    prefix                 (QStringLiteral("Ui.SendMessage.")),
    sendTo                 (prefix+QStringLiteral("sendTo")),
    unit                   (prefix+QStringLiteral("unit")),
    function               (prefix+QStringLiteral("function")),
    defaultAddress         (prefix+QStringLiteral("defaultAddress")),
    defaultFormat          (prefix+QStringLiteral("defaultFormat")),
    defaultCount           (prefix+QStringLiteral("defaultCount")),
    defaultData            (prefix+QStringLiteral("defaultData")),
    rwMultiRegWriteAddress (prefix+QStringLiteral("rwMultiRegWriteAddress")),
    rwMultiRegWriteFormat  (prefix+QStringLiteral("rwMultiRegWriteFormat")),
    rwMultiRegWriteCount   (prefix+QStringLiteral("rwMultiRegWriteCount")),
    rwMultiRegWriteData    (prefix+QStringLiteral("rwMultiRegWriteData")),
    rwMultiRegReadAddress  (prefix+QStringLiteral("rwMultiRegReadAddress")),
    rwMultiRegReadFormat   (prefix+QStringLiteral("rwMultiRegReadFormat")),
    rwMultiRegReadCount    (prefix+QStringLiteral("rwMultiRegReadCount")),
    rwMultiRegReadData     (prefix+QStringLiteral("rwMultiRegReadData")),
    writeMaskAddress       (prefix+QStringLiteral("writeMaskAddress")),
    writeMaskAnd           (prefix+QStringLiteral("writeMaskAnd")),
    writeMaskOr            (prefix+QStringLiteral("writeMaskOr")),
    readDeviceId           (prefix+QStringLiteral("readDeviceId")),
    readDeviceObjectId     (prefix+QStringLiteral("readDeviceObjectId")),
    readDeviceFormat       (prefix+QStringLiteral("readDeviceFormat")),
    list                   (prefix+QStringLiteral("list")),
    period                 (prefix+QStringLiteral("period"))
{
}

const mbClientSendMessageUi::Strings &mbClientSendMessageUi::Strings::instance()
{
    static const Strings s;
    return s;
}

QString mbClientSendMessageUi::getEventLogDescription(uint8_t eventId)
{
    QString res;
    if (eventId & 0x80)
    {
        res = "Receive Event: ";
        QStringList ls;
        if (eventId & 0x02) ls.append("Communication Error");
        if (eventId & 0x10) ls.append("Character Overrun");
        if (eventId & 0x20) ls.append("Currently in Listen Only Mode");
        if (eventId & 0x40) ls.append("Broadcast Received");
        res += ls.join(", ");
    }
    else if (eventId & 0x40)
    {
        res = "Send Event: ";
        QStringList ls;
        if (eventId & 0x01) ls.append("Read Exception Sent");
        if (eventId & 0x02) ls.append("Server Abort Exception Sent ");
        if (eventId & 0x04) ls.append("Server Busy Exception Sent");
        if (eventId & 0x08) ls.append("Server Program NAK Exception Sent");
        if (eventId & 0x10) ls.append("Write Timeout Error Occurred");
        if (eventId & 0x20) ls.append("Currently in Listen Only Mode");
        res += ls.join(", ");
    }
    else if (eventId == 0x04)
        res = "Entered Listen Only Mode";
    else if (eventId == 0x00)
        res = "Initiated Communication Restart";
    return res;
}

mbClientSendMessageUi::mbClientSendMessageUi(QWidget *parent) : mbCoreDialogBase(Strings::instance().prefix, parent),
                                                                ui(new Ui::mbClientSendMessageUi)
{
    ui->setupUi(this);
    m_list = new mbClientSendMessageListModel(this);
    ui->lsList->setModel(m_list);
    m_project = nullptr;
    m_sendTo = -1;
    m_timer = 0;

    QStringList ls;
    QComboBox *cmb;
    QSpinBox *sp;

    mbClient *core = mbClient::global();

    // -----------------------------------------------------------------------
    // Unit
    sp = ui->spUnit;
    sp->setMinimum(0);
    sp->setMaximum(255);

    // Read Data
    // Address type
    // Note: need to initialize earlier because it's used in setCurrentFuncIndex
    m_defaultAddress = new mbCoreAddressWidget();
    m_defaultAddress->setEnabledAddressType(false);
    ui->formLayoutDefaultParams->setWidget(1, QFormLayout::FieldRole, m_defaultAddress);

    // Formats
    ls = mb::enumFormatKeyList();
    Q_FOREACH (const QString &s, ls)
    {
        ui->cmbDefaultFormat        ->addItem(s);
        ui->cmbRWMultiRegWriteFormat->addItem(s);
        ui->cmbRWMultiRegReadFormat ->addItem(s);
        ui->cmbReadDataFormat       ->addItem(s);
        ui->cmbDiagnFormat          ->addItem(s);
        ui->cmbFileRecordDataFormat ->addItem(s);
        ui->cmbFIFOFormat           ->addItem(s);
        ui->cmbReadDeviceFormat     ->addItem(s);
    }
    ui->cmbDefaultFormat        ->setCurrentIndex(mb::Dec16);
    ui->cmbRWMultiRegWriteFormat->setCurrentIndex(mb::Dec16);
    ui->cmbRWMultiRegReadFormat ->setCurrentIndex(mb::Dec16);
    ui->cmbReadDataFormat       ->setCurrentIndex(mb::Dec16);
    ui->cmbDiagnFormat          ->setCurrentIndex(mb::Dec16);
    ui->cmbFileRecordDataFormat ->setCurrentIndex(mb::Dec16);
    ui->cmbFIFOFormat           ->setCurrentIndex(mb::Dec16);
    ui->cmbReadDeviceFormat     ->setCurrentIndex(mb::String);

    sp = ui->spDefaultCount;
    sp->setMinimum(1);
    sp->setMaximum(MB_MAX_DISCRETS); // TODO: if register was choosen than change this value

    // -----------------------------------------------------------------------
    // Read/Write Multiple Registers
    // Address type
    // Note: need to initialize earlier because it's used in setCurrentFuncIndex
    m_rwMultiRegWriteAddress = new mbCoreAddressWidget();
    m_rwMultiRegWriteAddress->setEnabledAddressType(false);
    ui->formLayoutRWMultiRegWriteParams->setWidget(1, QFormLayout::FieldRole, m_rwMultiRegWriteAddress);

    sp = ui->spRWMultiRegWriteCount;
    sp->setMinimum(1);
    sp->setMaximum(MB_MAX_REGISTERS); // TODO: if register was choosen than change this value

    m_rwMultiRegReadAddress = new mbCoreAddressWidget();
    m_rwMultiRegReadAddress->setEnabledAddressType(false);
    ui->formLayoutRWMultiRegReadParams->setWidget(1, QFormLayout::FieldRole, m_rwMultiRegReadAddress);

    // ReadFormat
    sp = ui->spRWMultiRegReadCount;
    sp->setMinimum(1);
    sp->setMaximum(MB_MAX_REGISTERS); // TODO: if register was choosen than change this value

    sp = ui->spPeriod;
    sp->setMinimum(1);
    sp->setMaximum(INT_MAX);
    sp->setValue(1000);

    m_writeMaskAddress = new mbCoreAddressWidget();
    m_writeMaskAddress->setEnabledAddressType(false);
    ui->formLayoutWriteMaskParams->setWidget(0, QFormLayout::FieldRole, m_writeMaskAddress);

    sp = ui->spWriteMaskAnd;
    sp->setMinimum(0);
    sp->setMaximum(USHRT_MAX);
    sp->setDisplayIntegerBase(16);
    sp->setValue(0);

    sp = ui->spWriteMaskOr;
    sp->setMinimum(0);
    sp->setMaximum(USHRT_MAX);
    sp->setDisplayIntegerBase(16);
    sp->setValue(0);

    cmb = ui->cmbFunction;
    connect(cmb, SIGNAL(currentIndexChanged(int)), this, SLOT(setCurrentFuncIndex(int)));
    m_funcNums.append(MBF_READ_COILS                      );
    m_funcNums.append(MBF_READ_DISCRETE_INPUTS            );
    m_funcNums.append(MBF_READ_HOLDING_REGISTERS          );
    m_funcNums.append(MBF_READ_INPUT_REGISTERS            );
    m_funcNums.append(MBF_WRITE_SINGLE_COIL               );
    m_funcNums.append(MBF_WRITE_SINGLE_REGISTER           );
    m_funcNums.append(MBF_READ_EXCEPTION_STATUS           );
    m_funcNums.append(MBF_DIAGNOSTICS                     );
    m_funcNums.append(MBF_GET_COMM_EVENT_COUNTER          );
    m_funcNums.append(MBF_GET_COMM_EVENT_LOG              );
    m_funcNums.append(MBF_WRITE_MULTIPLE_COILS            );
    m_funcNums.append(MBF_WRITE_MULTIPLE_REGISTERS        );
    m_funcNums.append(MBF_REPORT_SERVER_ID                );
    m_funcNums.append(MBF_READ_FILE_RECORD                );
    m_funcNums.append(MBF_WRITE_FILE_RECORD               );
    m_funcNums.append(MBF_MASK_WRITE_REGISTER             );
    m_funcNums.append(MBF_READ_WRITE_MULTIPLE_REGISTERS   );
    m_funcNums.append(MBF_READ_FIFO_QUEUE                 );
    Q_FOREACH (uint8_t funcNum, m_funcNums)
    {
        cmb->addItem(QString("%1 - %2")
                         .arg(funcNum, 2, 10, QChar('0'))
                         .arg(mb::ModbusFunctionString(funcNum))
                     );
    }
    m_funcNums.append(MBF_ENCAPSULATED_INTERFACE_TRANSPORT);
    cmb->addItem(QString("%1/%2 - ReadDeviceIdentification")
                    .arg(MBF_ENCAPSULATED_INTERFACE_TRANSPORT, 2, 10, QChar('0'))
                    .arg(MBF_MEI_READ_DEVICE_ID, 2, 10, QChar('0')));
    cmb->setCurrentIndex(2);

    cmb = ui->cmbDiagnSubfunction;
    connect(cmb, SIGNAL(currentIndexChanged(int)), this, SLOT(setCurrentDiagnSubfuncIndex(int)));
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
        cmb->addItem(QString("%1 - %2")
                         .arg(funcNum, 2, 10, QChar('0'))
                         .arg(mb::ModbusDiagnSubfunctionString(funcNum))
                     );
    }
    cmb->setCurrentIndex(2);

    // GetCommEventCounter
    //ui->lnGetCommEventCounterStatus->setText("0000");
    //ui->lnGetCommEventCounterCount->setText("0");

    // GetCommEventLog
    //ui->lnGetCommEventLogStatus->setText("0000");
    //ui->lnGetCommEventLogEventCount->setText("0");
    //ui->lnGetCommEventLogMessageCount->setText("0");

    // Read/Write File Records
    m_fileRecordModel = new mbClientSendMessageFileRecordModel(this);
    ui->tblFileRecords->setModel(m_fileRecordModel);

    connect(ui->btnFileRecordAdd     , &QPushButton::clicked, this, &mbClientSendMessageUi::slotFileRecordAdd     );
    connect(ui->btnFileRecordDelete  , &QPushButton::clicked, this, &mbClientSendMessageUi::slotFileRecordDelete  );
    connect(ui->btnFileRecordMoveUp  , &QPushButton::clicked, this, &mbClientSendMessageUi::slotFileRecordMoveUp  );
    connect(ui->btnFileRecordMoveDown, &QPushButton::clicked, this, &mbClientSendMessageUi::slotFileRecordMoveDown);
    connect(ui->btnFileRecordClear   , &QPushButton::clicked, this, &mbClientSendMessageUi::slotFileRecordClear   );

    // Read FIFO queue
    sp = ui->spFIFOOffset;
    sp->setMinimum(0x0000);
    sp->setMaximum(0xFFFF);
    sp->setValue(0);

    // Read Device Identification
    sp = ui->spReadDeviceId;
    sp->setMinimum(0);
    sp->setMaximum(UINT8_MAX);
    sp->setValue(1);
 
    sp = ui->spReadDeviceObjectId;
    sp->setMinimum(0);
    sp->setMaximum(UINT8_MAX);
    sp->setValue(0);

    ui->lnReadDeviceConformity->setText("0");
    ui->lnReadDeviceNextObjectId->setText("0");

    connect(mbCore::globalCore(), &mbCore::addressNotationChanged, this, &mbClientSendMessageUi::setModbusAddresNotation);

    setSendTo(SendToDevice);

    // Connect RadioButtons SendTo
    connect(ui->rdDevice, &QRadioButton::clicked, this, [this]() {
        this->setSendTo(SendToDevice);
    });
    connect(ui->rdPortUnit, &QRadioButton::clicked, this, [this]() {
        this->setSendTo(SendToPortUnit);
    });


    connect(ui->btnListShowHide, &QPushButton::clicked, this, &mbClientSendMessageUi::slotListShowHide);
    connect(ui->btnListInsert  , &QPushButton::clicked, this, &mbClientSendMessageUi::slotListInsert  );
    connect(ui->btnListEdit    , &QPushButton::clicked, this, &mbClientSendMessageUi::slotListEdit    );
    connect(ui->btnListRemove  , &QPushButton::clicked, this, &mbClientSendMessageUi::slotListRemove  );
    connect(ui->btnListClear   , &QPushButton::clicked, this, &mbClientSendMessageUi::slotListClear   );
    connect(ui->btnListMoveUp  , &QPushButton::clicked, this, &mbClientSendMessageUi::slotListMoveUp  );
    connect(ui->btnListMoveDown, &QPushButton::clicked, this, &mbClientSendMessageUi::slotListMoveDown);
    connect(ui->btnListImport  , &QPushButton::clicked, this, &mbClientSendMessageUi::slotListImport  );
    connect(ui->btnListExport  , &QPushButton::clicked, this, &mbClientSendMessageUi::slotListExport  );

    ui->btnListShowHide->click();

    connect(ui->btnSendOne  , &QPushButton::clicked, this, &mbClientSendMessageUi::slotSendOne );
    connect(ui->btnSendList , &QPushButton::clicked, this, &mbClientSendMessageUi::slotSendList);
    connect(ui->btnStop     , &QPushButton::clicked, this, &mbClientSendMessageUi::slotStop    );
    connect(ui->btnClose    , &QPushButton::clicked, this, &QDialog::close);

    connect(ui->lsList, &QAbstractItemView::doubleClicked, this, &mbClientSendMessageUi::getListItem);

    connect(core, &mbClient::projectChanged, this, &mbClientSendMessageUi::setProject);
    connect(core, &mbClient::statusChanged , this, &mbClientSendMessageUi::setRunStatus);
    setProject(core->project());
}

mbClientSendMessageUi::~mbClientSendMessageUi()
{
    delete ui;
}

MBSETTINGS mbClientSendMessageUi::cachedSettings() const
{
    const Strings &s = Strings::instance();

    MBSETTINGS m = mbCoreDialogBase::cachedSettings();
    m[s.sendTo                ] = QMetaEnum::fromType<SendTo>().valueToKey(m_sendTo);
    m[s.unit                  ] = ui->spUnit->value();
    m[s.function              ] = getCurrentFuncNum();
    m[s.defaultAddress        ] = getDefaultAddress      ();
    m[s.defaultFormat         ] = ui->cmbDefaultFormat         ->currentText();
    m[s.defaultCount          ] = ui->spDefaultCount           ->value      ();
    m[s.defaultData           ] = ui->txtDefaultData           ->toPlainText();
    m[s.rwMultiRegWriteAddress] = getRWMultiRegWriteAddress();
    m[s.rwMultiRegWriteFormat ] = ui->cmbRWMultiRegWriteFormat ->currentText();
    m[s.rwMultiRegWriteCount  ] = ui->spRWMultiRegWriteCount   ->value      ();
    m[s.rwMultiRegWriteData   ] = ui->txtRWMultiRegWriteData   ->toPlainText();
    m[s.rwMultiRegReadAddress ] = getRWMultiRegReadAddress();
    m[s.rwMultiRegReadFormat  ] = ui->cmbRWMultiRegReadFormat  ->currentText();
    m[s.rwMultiRegReadCount   ] = ui->spRWMultiRegReadCount    ->value      ();
    m[s.rwMultiRegReadData    ] = ui->txtRWMultiRegReadData    ->toPlainText();
    m[s.writeMaskAddress      ] = getWriteMaskAddress      ();
    m[s.writeMaskAnd          ] = ui->spWriteMaskAnd           ->value      ();
    m[s.writeMaskOr           ] = ui->spWriteMaskOr            ->value      ();
    m[s.readDeviceId          ] = ui->spReadDeviceId           ->value      ();
    m[s.readDeviceObjectId    ] = ui->spReadDeviceObjectId     ->value      ();
    m[s.readDeviceFormat      ] = ui->cmbReadDeviceFormat      ->currentText();
    m[s.list                  ] = this                         ->getListItems();
    m[s.period                ] = ui->spPeriod                 ->value      ();
    m[s.wGeometry             ] = this->saveGeometry();

    return m;
}

void mbClientSendMessageUi::setCachedSettings(const MBSETTINGS &m)
{
    mbCoreDialogBase::setCachedSettings(m);

    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();
    //bool ok;

    it = m.find(s.sendTo                ); if (it != end) setSendTo(mb::enumValue<SendTo>(it.value()));
    it = m.find(s.unit                  ); if (it != end) ui->spUnit->setValue(it.value().toInt());
    it = m.find(s.function              ); if (it != end) setCurrentFuncNum(static_cast<uint8_t> (it.value().toInt())  );
    it = m.find(s.defaultAddress        ); if (it != end) setDefaultAddress                      (it.value().toInt()   );
    it = m.find(s.defaultFormat         ); if (it != end) ui->cmbDefaultFormat  ->setCurrentText (it.value().toString());
    it = m.find(s.defaultCount          ); if (it != end) ui->spDefaultCount    ->setValue       (it.value().toInt()   );
    it = m.find(s.defaultData           ); if (it != end) ui->txtDefaultData    ->setPlainText   (it.value().toString());
    it = m.find(s.rwMultiRegWriteAddress); if (it != end) setRWMultiRegWriteAddress              (it.value().toInt()   );
    it = m.find(s.rwMultiRegWriteFormat ); if (it != end) ui->cmbDefaultFormat  ->setCurrentText (it.value().toString());
    it = m.find(s.rwMultiRegWriteCount  ); if (it != end) ui->spDefaultCount    ->setValue       (it.value().toInt()   );
    it = m.find(s.rwMultiRegWriteData   ); if (it != end) ui->txtDefaultData    ->setPlainText   (it.value().toString());
    it = m.find(s.rwMultiRegReadAddress ); if (it != end) setRWMultiRegReadAddress               (it.value().toInt()   );
    it = m.find(s.rwMultiRegReadFormat  ); if (it != end) ui->cmbDefaultFormat  ->setCurrentText (it.value().toString());
    it = m.find(s.rwMultiRegReadCount   ); if (it != end) ui->spDefaultCount    ->setValue       (it.value().toInt()   );
    it = m.find(s.rwMultiRegReadData    ); if (it != end) ui->txtDefaultData    ->setPlainText   (it.value().toString());
    it = m.find(s.writeMaskAddress      ); if (it != end) setWriteMaskAddress                    (it.value().toInt()   );
    it = m.find(s.writeMaskAnd          ); if (it != end) ui->spWriteMaskAnd    ->setValue       (it.value().toInt()   );
    it = m.find(s.writeMaskOr           ); if (it != end) ui->spWriteMaskOr     ->setValue       (it.value().toInt()   );
    it = m.find(s.readDeviceId          ); if (it != end) ui->spReadDeviceId    ->setValue       (it.value().toInt()   );
    it = m.find(s.readDeviceObjectId    ); if (it != end) ui->spReadDeviceObjectId->setValue       (it.value().toInt()   );
    it = m.find(s.readDeviceFormat      ); if (it != end) ui->cmbReadDeviceFormat->setCurrentText (it.value().toString());
    it = m.find(s.list                  ); if (it != end) this                  ->setListItems   (it.value().toStringList()     );
    it = m.find(s.period                ); if (it != end) ui->spPeriod          ->setValue       (it.value().toInt()   );
    it = m.find(s.wGeometry             ); if (it != end) this                  ->restoreGeometry(it.value().toByteArray());
}

void mbClientSendMessageUi::setModbusAddresNotation(mb::AddressNotation notation)
{
    m_defaultAddress        ->setAddressNotation(notation);
    m_rwMultiRegWriteAddress->setAddressNotation(notation);
    m_rwMultiRegReadAddress ->setAddressNotation(notation);
    m_writeMaskAddress      ->setAddressNotation(notation);
}

void mbClientSendMessageUi::setProject(mbCoreProject *p)
{
    mbClientProject *project = static_cast<mbClientProject*>(p);
    if (m_project != project)
    {
        if (m_project)
        {
            m_project->disconnect(this);
            ui->cmbPort->clear();
            ui->cmbDevice->clear();
        }
        m_project = project;
        if (m_project)
        {
            QList<mbClientPort*> ports = m_project->ports();
            connect(m_project, &mbClientProject::portAdded   , this, &mbClientSendMessageUi::addPort   );
            connect(m_project, &mbClientProject::portRemoving, this, &mbClientSendMessageUi::removePort);
            connect(m_project, &mbClientProject::portRenaming, this, &mbClientSendMessageUi::renamePort);
            Q_FOREACH (mbClientPort *d, ports)
                addPort(d);

            QList<mbClientDevice*> devices = m_project->devices();
            connect(m_project, &mbClientProject::deviceAdded   , this, &mbClientSendMessageUi::addDevice   );
            connect(m_project, &mbClientProject::deviceRemoving, this, &mbClientSendMessageUi::removeDevice);
            connect(m_project, &mbClientProject::deviceRenaming, this, &mbClientSendMessageUi::renameDevice);
            Q_FOREACH (mbClientDevice *d, devices)
                addDevice(d);
        }
    }
}

void mbClientSendMessageUi::addPort(mbCorePort *port)
{
    int i = m_project->portIndex(port);
    ui->cmbPort->insertItem(i, port->name());
}

void mbClientSendMessageUi::removePort(mbCorePort *port)
{
    int i = m_project->portIndex(port);
    ui->cmbPort->removeItem(i);
}

void mbClientSendMessageUi::renamePort(mbCorePort *port, const QString newName)
{
    int i = m_project->portIndex(port);
    ui->cmbPort->setItemText(i, newName);
}

void mbClientSendMessageUi::addDevice(mbCoreDevice *device)
{
    int i = m_project->deviceIndex(device);
    ui->cmbDevice->insertItem(i, device->name());
}

void mbClientSendMessageUi::removeDevice(mbCoreDevice *device)
{
    int i = m_project->deviceIndex(device);
    ui->cmbDevice->removeItem(i);
}

void mbClientSendMessageUi::renameDevice(mbCoreDevice *device, const QString newName)
{
    int i = m_project->deviceIndex(device);
    ui->cmbDevice->setItemText(i, newName);
}

void mbClientSendMessageUi::setCurrentFuncIndex(int funcIndex)
{
    uint8_t funcNum = m_funcNums.value(funcIndex);
    setCurrentFuncNum(funcNum);
}

void mbClientSendMessageUi::setCurrentDiagnSubfuncIndex(int funcIndex)
{
    uint8_t funcNum = m_diagnSubfuncNums.value(funcIndex);
    setCurrentDiagnSubfuncNum(funcNum);
}

void mbClientSendMessageUi::setRunStatus(int status)
{
    if (status == mbClient::Stopped)
        slotStop();
}

void mbClientSendMessageUi::slotFileRecordAdd()
{
    int i = currentFileRecordIndex();
    m_fileRecordModel->insertRecord(i);
}

void mbClientSendMessageUi::slotFileRecordDelete()
{
    int i = currentFileRecordIndex();
    m_fileRecordModel->removeRecord(i);
}

void mbClientSendMessageUi::slotFileRecordMoveUp()
{
    int i = currentFileRecordIndex();
    m_fileRecordModel->moveUp(i);
}

void mbClientSendMessageUi::slotFileRecordMoveDown()
{
    int i = currentFileRecordIndex();
    m_fileRecordModel->moveDown(i);
}
void mbClientSendMessageUi::slotFileRecordClear()
{
    m_fileRecordModel->clear();
}

void mbClientSendMessageUi::slotListShowHide()
{
    bool isVisible = ui->grList->isVisible();
    if (isVisible)
        ui->btnListShowHide->setText("Show");
    else
        ui->btnListShowHide->setText("Hide");
    ui->grList->setVisible(!isVisible);
}

void mbClientSendMessageUi::slotListInsert()
{
    mbClientMessageParams params;
    fillParams(params);
    int i = currentListIndex();
    m_list->insertMessage(i, params);
}

void mbClientSendMessageUi::slotListEdit()
{
    int i = currentListIndex();
    if (i < 0)
        return;
    mbClientMessageParams params;
    fillParams(params);
    m_list->editMessage(i, params);
}

void mbClientSendMessageUi::slotListRemove()
{
    int i = currentListIndex();
    m_list->removeMessage(i);
}

void mbClientSendMessageUi::slotListClear()
{
    m_list->clear();
}

void mbClientSendMessageUi::slotListMoveUp()
{
    int i = currentListIndex();
    if (m_list->moveUp(i))
        setCurrentListIndex(i-1);
}

void mbClientSendMessageUi::slotListMoveDown()
{
    int i = currentListIndex();
    if (m_list->moveDown(i))
        setCurrentListIndex(i+1);
}

void mbClientSendMessageUi::slotListImport()
{
    mbClientDialogs *dialogs = mbClient::global()->ui()->dialogs();
    QString file = dialogs->getOpenFileName(this,
                                            QString("Import Message"),
                                            QString(),
                                            dialogs->getFilterString(mbCoreDialogs::Filter_AllFiles));
    if (!file.isEmpty())
    {
        QFile qf(file);
        if (qf.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QTextStream in(&qf);
            QList<mbClientMessageParams> messages;
            bool ok = false;
            while (!in.atEnd())
            {
                QString line = in.readLine();
                mbClientMessageParams m = mb::restoreClientMessageParams(line, &ok);
                if (ok)
                    messages.append(m);
            }
            qf.close();
            m_list->setMessages(messages);
        }
    }
}

void mbClientSendMessageUi::slotListExport()
{
    mbClientDialogs *dialogs = mbClient::global()->ui()->dialogs();
    QString file = dialogs->getSaveFileName(this,
                                            QString("Export Message"),
                                            QString(),
                                            dialogs->getFilterString(mbCoreDialogs::Filter_AllFiles));
    if (!file.isEmpty())
    {
        QFile qf(file);
        if (qf.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            QTextStream out(&qf);
            QList<mbClientMessageParams> messages = m_list->messages();
            Q_FOREACH (const mbClientMessageParams &m, messages)
            {
                QString line = mb::saveClientMessageParams(m);
                out << line << '\n';
            }
            qf.close();
        }
    }
}

void mbClientSendMessageUi::slotSendOne()
{
    mbClient *core = mbClient::global();
    if (!core->isRunning())
        core->start();
    if (!prepareSendParams())
        return;
    createMessage();
    if (m_messageList.count())
    {
        startSendMessages();
    }
}

void mbClientSendMessageUi::slotSendList()
{
    mbClient *core = mbClient::global();
    if (!core->isRunning())
        core->start();
    if (!prepareSendParams())
        return;
    createMessageList();
    if (m_messageList.count())
    {
        startSendMessages();
    }
}

void mbClientSendMessageUi::slotStop()
{
    stopSendMessages();
}
#include <QDebug>
void mbClientSendMessageUi::slotBytesTx(const QByteArray &bytes)
{
    qDebug() << "mbClientSendMessageUi::slotBytesTx:" << Modbus::bytesToString(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.length()).data();
    ui->txtModbusTx->setPlainText(Modbus::bytesToString(bytes));
}

void mbClientSendMessageUi::slotBytesRx(const QByteArray &bytes)
{
    ui->txtModbusRx->setPlainText(Modbus::bytesToString(bytes));
}

void mbClientSendMessageUi::slotAsciiTx(const QByteArray &bytes)
{
    ui->txtModbusTx->setPlainText(Modbus::asciiToString(bytes));
}

void mbClientSendMessageUi::slotAsciiRx(const QByteArray &bytes)
{
    ui->txtModbusRx->setPlainText(Modbus::asciiToString(bytes));
}

void mbClientSendMessageUi::messageCompleted()
{
    mbClientRunMessagePtr message = qobject_cast<mbClientRunMessage*>(sender());
    if (message)
    {
        ui->lnStatus   ->setText(mb::toString(message->status   ()));
        ui->lnTimestamp->setText(mb::toString(message->timestamp()));
        fillForm(message);
        if (isTimerStopped())
            stopSendMessages();
    }
}

void mbClientSendMessageUi::getListItem(const QModelIndex &index)
{
    int i = index.row();
    if (i < 0)
        return;
    // TODO: check if txt already has data and ask user to confirm
    mbClientMessageParams p = m_list->message(i);
    fillForm(p);
}

void mbClientSendMessageUi::timerEvent(QTimerEvent */*event*/)
{
    mbClientRunMessagePtr msg = m_messageList.value(m_messageIndex);
    if (msg && msg->isCompleted())
    {
        msg->clearCompleted();
        ++m_messageIndex;
        if (m_messageIndex >= m_messageList.count())
        {
            if (ui->chbUseLoop->isChecked())
                m_messageIndex = 0;
            else
            {
                stopSendMessages();
                return;
            }
        }
        this->sendMessage();
    }
}

int mbClientSendMessageUi::currentFileRecordIndex() const
{
    auto indexes = ui->tblFileRecords->selectionModel()->selectedIndexes();
    if (indexes.count())
        return indexes.first().row();
    return -1;
}

QStringList mbClientSendMessageUi::getListItems() const
{
    return mb::saveClientMessages(m_list->messages());
}

void mbClientSendMessageUi::setListItems(const QStringList &list)
{
    m_list->setMessages(mb::restoreClientMessages(list));
}

int mbClientSendMessageUi::currentListIndex() const
{
    auto indexes = ui->lsList->selectionModel()->selectedIndexes();
    if (indexes.count())
        return indexes.first().row();
    return -1;
}

void mbClientSendMessageUi::setCurrentListIndex(int i)
{
    QModelIndex index = m_list->index(i, 0);
    ui->lsList->selectionModel()->select(index, QItemSelectionModel::ClearAndSelect);
}

void mbClientSendMessageUi::createMessage()
{
    m_messageIndex = 0;
    m_messageList.clear();
    mbClientMessageParams params;
    fillParams(params);
    mbClientRunMessage *msg = this->createMessage(params);
    m_messageList.append(msg);
}

void mbClientSendMessageUi::createMessageList()
{
    m_messageIndex = 0;
    m_messageList.clear();
    auto msgList = m_list->messages();
    if (msgList.count() == 0)
        return;
    Q_FOREACH (const auto& params, msgList)
    {
        mbClientRunMessage *msg = this->createMessage(params);
        m_messageList.append(msg);
    }
}

mbClientRunMessage *mbClientSendMessageUi::createMessage(const mbClientMessageParams &params)
{
    mbClientDevice *device = currentDevice();
    if (!device)
        return nullptr;
    QByteArray data;
    uint16_t c = 0;
    mbClientRunMessage *msg;
    switch(params.func)
    {
    case MBF_READ_COILS:
        return new mbClientRunMessageReadCoils(params.offset,
                                               params.count,
                                               m_dataParams.maxReadCoils,
                                               this);
    case MBF_READ_DISCRETE_INPUTS:
        return new mbClientRunMessageReadDiscreteInputs(params.offset,
                                                        params.count,
                                                        m_dataParams.maxReadDiscreteInputs,
                                                        this);
    case MBF_READ_HOLDING_REGISTERS:
        return new mbClientRunMessageReadHoldingRegisters(params.offset,
                                                          params.count,
                                                          m_dataParams.maxReadHoldingRegisters,
                                                          this);
    case MBF_READ_INPUT_REGISTERS:
        return new mbClientRunMessageReadInputRegisters(params.offset,
                                                        params.count,
                                                        m_dataParams.maxReadInputRegisters,
                                                        this);
    case MBF_WRITE_SINGLE_COIL:
    {
        msg = new mbClientRunMessageWriteSingleCoil(params.offset,
                                                    this);
        char v = (params.data == QStringLiteral("1"));
        data = QByteArray(v, 1);
        c = 1;
    }
        break;
    case MBF_WRITE_SINGLE_REGISTER:
    {
        msg = new mbClientRunMessageWriteSingleRegister(params.offset,
                                                        this);
        switch (params.format)
        {
        case mb::Bool:
        {
            QStringList ls = dataToStringList(params.data);
            data = fromStringListBits(ls);
        }
            break;
        case mb::ByteArray:
        case mb::String:
            data = mb::toByteArray(params.data,
                                   params.format,
                                   Modbus::Memory_4x,
                                   m_dataParams.swapBytes,
                                   m_dataParams.registerOrder,
                                   m_dataParams.byteArrayFormat,
                                   m_dataParams.stringEncoding,
                                   m_dataParams.stringLengthType,
                                   m_dataParams.byteArraySeparator,
                                   msg->count()*2);
            break;
        default:
        {
            QStringList ls = dataToStringList(params.data);
            data = fromStringListNumbers(ls, params.format);
        }
            break;
        }
    }
        break;
    case MBF_READ_EXCEPTION_STATUS:
        return new mbClientRunMessageReadExceptionStatus(this);
    case MBF_DIAGNOSTICS:
        switch (params.subfunc)
        {
        case MBF_DIAGNOSTICS_RETURN_QUERY_DATA:
            switch (params.format)
            {
            case mb::Bool:
            {
                QStringList ls = dataToStringList(params.data);
                data = fromStringListBits(ls);
            }
                break;
            case mb::ByteArray:
            case mb::String:
                data = mb::toByteArray(params.data,
                                       params.format,
                                       Modbus::Memory_0x,
                                       m_dataParams.swapBytes,
                                       m_dataParams.registerOrder,
                                       m_dataParams.byteArrayFormat,
                                       m_dataParams.stringEncoding,
                                       m_dataParams.stringLengthType,
                                       m_dataParams.byteArraySeparator,
                                       0);
                break;
            default:
            {
                QStringList ls = dataToStringList(params.data);
                data = fromStringListNumbers(ls, params.format);
            }
                break;
            }
            break;
        case MBF_DIAGNOSTICS_RESTART_COMMUNICATIONS_OPTION:
            if (params.data.toInt())
                data = QByteArray("\xFF\x00", 2);
            else
                data = QByteArray("\0\0", 2);
            break;
        case MBF_DIAGNOSTICS_CHANGE_ASCII_INPUT_DELIMITER:
            if (params.data.length())
            {
                char d[2];
                d[0] = params.data.front().toLatin1();
                d[1] = '\0';
                data = QByteArray(d, 2);
            }
            else
            {
                data = QByteArray("\0\0", 2);
            }
            break;
        default:
            data = QByteArray("\0\0", 2);
            break;
        }
        return new mbClientRunMessageDiagnostics(params.subfunc, data.constData(), data.size(), this);
    case MBF_GET_COMM_EVENT_COUNTER:
        return new mbClientRunMessageGetCommEventCounter(this);
    case MBF_GET_COMM_EVENT_LOG:
        return new mbClientRunMessageGetCommEventLog(this);
    case MBF_WRITE_MULTIPLE_COILS:
    {
        msg = new mbClientRunMessageWriteMultipleCoils(params.offset,
                                                       params.count,
                                                       m_dataParams.maxWriteMultipleCoils,
                                                       this);
        switch (params.format)
        {
        case mb::Bool:
        {
            QStringList ls = dataToStringList(params.data);
            data = fromStringListBits(ls);
            c = ls.count();
        }
        break;
        case mb::ByteArray:
        case mb::String:
            data = mb::toByteArray(params.data,
                                   params.format,
                                   Modbus::Memory_0x,
                                   m_dataParams.swapBytes,
                                   m_dataParams.registerOrder,
                                   m_dataParams.byteArrayFormat,
                                   m_dataParams.stringEncoding,
                                   m_dataParams.stringLengthType,
                                   m_dataParams.byteArraySeparator,
                                   (msg->count()+7)/8);
            c = data.count() * 8;
            break;
        default:
        {
            QStringList ls = dataToStringList(params.data);
            data = fromStringListNumbers(ls, params.format);
            c = data.count() * 8;
        }
        break;
        }
    }
        break;
    case MBF_WRITE_MULTIPLE_REGISTERS:
    {
        msg = new mbClientRunMessageWriteMultipleRegisters(params.offset,
                                                           params.count,
                                                           m_dataParams.maxWriteMultipleRegisters,
                                                           this);
        switch (params.format)
        {
        case mb::Bool:
        {
            QStringList ls = dataToStringList(params.data);
            data = fromStringListBits(ls);
        }
        break;
        case mb::ByteArray:
        case mb::String:
            data = mb::toByteArray(params.data,
                                   params.format,
                                   Modbus::Memory_4x,
                                   m_dataParams.swapBytes,
                                   m_dataParams.registerOrder,
                                   m_dataParams.byteArrayFormat,
                                   m_dataParams.stringEncoding,
                                   m_dataParams.stringLengthType,
                                   m_dataParams.byteArraySeparator,
                                   msg->count()*2);
            break;
        default:
        {
            QStringList ls = dataToStringList(params.data);
            data = fromStringListNumbers(ls, params.format);
        }
            break;
        }
        if (data.count() && (data.count() & 1))
            data.append('\0');
        c = data.count() / 2;
    }
        break;
    case MBF_REPORT_SERVER_ID:
       return new mbClientRunMessageReportServerID(this);
    case MBF_READ_FILE_RECORD:
        return nullptr; //new mbClientRunMessageReportServerID(this);
    case MBF_WRITE_FILE_RECORD:
        return nullptr; //new mbClientRunMessageReportServerID(this);
    case MBF_MASK_WRITE_REGISTER:
    {
        msg = new mbClientRunMessageMaskWriteRegister(params.offset, this);
        struct { quint16 andMask; quint16 orMask; } v;
        v.andMask = static_cast<quint16>(ui->spWriteMaskAnd->value());
        v.orMask  = static_cast<quint16>(ui->spWriteMaskOr ->value());
        data = QByteArray(reinterpret_cast<char*>(&v), sizeof(v));
        c = 2;
    }
        break;
    case MBF_READ_WRITE_MULTIPLE_REGISTERS:
    {
        msg = new mbClientRunMessageReadWriteMultipleRegisters(params.offset,
                                                               params.count,
                                                               params.writeOffset,
                                                               params.writeCount,
                                                               this);
        switch (params.format)
        {
        case mb::Bool:
        {
            QStringList ls = dataToStringList(params.data);
            data = fromStringListBits(ls);
        }
        break;
        case mb::ByteArray:
        case mb::String:
            data = mb::toByteArray(params.data,
                                   params.format,
                                   Modbus::Memory_4x,
                                   m_dataParams.swapBytes,
                                   m_dataParams.registerOrder,
                                   m_dataParams.byteArrayFormat,
                                   m_dataParams.stringEncoding,
                                   m_dataParams.stringLengthType,
                                   m_dataParams.byteArraySeparator,
                                   msg->count()*2);
            break;
        default:
        {
            QStringList ls = dataToStringList(params.data);
            data = fromStringListNumbers(ls, params.format);
        }
            break;
        }
        if (data.count() && (data.count() & 1))
            data.append('\0');
        c = data.count() / 2;
    }
        break;
    case MBF_READ_FIFO_QUEUE:
        return new mbClientRunMessageReadFIFOQueue(params.offset, this);
    case MBF_ENCAPSULATED_INTERFACE_TRANSPORT:
        return new mbClientRunMessageReadDeviceId(params.deviceId, params.objectId, this);
    default:
        return nullptr;
    }

    if (data.count())
    {
        msg->setData(0, c, data.data());
    }
    return msg;
}

void mbClientSendMessageUi::sendMessage()
{
    if (m_messageList.count() == 0)
        return;
    mbClientRunMessagePtr msg = m_messageList.value(m_messageIndex);
    if (msg)
    {
        //ui->txtModbusTx->clear();
        ui->txtModbusRx->clear();
        if (m_sendTo == SendToPortUnit)
        {
            mbClientPort *port = currentPort();
            if (!port)
                return;
            mbClient::global()->sendPortMessage(port->handle(), msg);
        }
        else
        {
            mbClientDevice *device = currentDevice();
            if (!device)
                return;
            mbClient::global()->sendMessage(device->handle(), msg);
        }
    }
}

void mbClientSendMessageUi::prepareToSend(mbClientRunMessage *msg)
{
    if (m_sendTo == SendToPortUnit)
        msg->setUnit(m_unit);
    connect(msg, &mbClientRunMessage::signalBytesTx, this, &mbClientSendMessageUi::slotBytesTx     );
    connect(msg, &mbClientRunMessage::signalBytesRx, this, &mbClientSendMessageUi::slotBytesRx     );
    connect(msg, &mbClientRunMessage::signalAsciiTx, this, &mbClientSendMessageUi::slotBytesTx     );
    connect(msg, &mbClientRunMessage::signalAsciiRx, this, &mbClientSendMessageUi::slotBytesRx     );
    connect(msg, &mbClientRunMessage::completed    , this, &mbClientSendMessageUi::messageCompleted);
    switch (msg->function())
    {
    case MBF_READ_COILS:
    case MBF_READ_DISCRETE_INPUTS:
    case MBF_READ_HOLDING_REGISTERS:
    case MBF_READ_INPUT_REGISTERS:
        ui->txtDefaultData->clear();
        break;
    case MBF_READ_EXCEPTION_STATUS:
    case MBF_REPORT_SERVER_ID:
        ui->txtReadData->clear();
        break;
    case MBF_DIAGNOSTICS:
        ui->txtDiagnResponse->clear();
        break;
    case MBF_GET_COMM_EVENT_COUNTER:
        ui->lnGetCommEventCounterStatus->clear();
        ui->lnGetCommEventCounterCount->clear();
        break;
    case MBF_GET_COMM_EVENT_LOG:
        ui->lnGetCommEventLogStatus->clear();
        ui->lnGetCommEventLogEventCount->clear();
        ui->lnGetCommEventLogMessageCount->clear();
        ui->tblEventLog->clearContents();
        break;
    case MBF_READ_FILE_RECORD:
        break;
    case MBF_WRITE_FILE_RECORD:
        break;
    case MBF_READ_WRITE_MULTIPLE_REGISTERS:
        ui->txtRWMultiRegReadData->clear();
        break;
    case MBF_READ_FIFO_QUEUE:
        ui->txtFIFOData->clear();
        break;
    case MBF_ENCAPSULATED_INTERFACE_TRANSPORT:
        ui->tblReadDeviceObjects->clearContents();
    }
}

void mbClientSendMessageUi::clearAfterSend(mbClientRunMessage *msg)
{
    msg->disconnect(this);
}

void mbClientSendMessageUi::startSendMessages()
{
    setEnableParams(false);
    ui->txtModbusTx->clear();
    ui->txtModbusRx->clear();
    Q_FOREACH (auto msg, m_messageList)
    {
        prepareToSend(msg.data());
    }
    sendMessage();
    if ((m_messageList.count() > 1 || ui->chbUseLoop->isChecked()) && isTimerStopped())
    {
        m_timer = startTimer(ui->spPeriod->value());;
    }
}

void mbClientSendMessageUi::stopSendMessages()
{
    if (isTimerRunning())
    {
        killTimer(m_timer);
        m_timer = 0;
    }
    Q_FOREACH (auto msg, m_messageList)
    {
        clearAfterSend(msg);
    }
    m_messageList.clear();
    setEnableParams(true);
}

mbClientPort *mbClientSendMessageUi::currentPort() const
{
    if (m_project)
    {
        int i = ui->cmbPort->currentIndex();
        return m_project->port(i);
    }
    return nullptr;
}

mbClientDevice *mbClientSendMessageUi::currentDevice() const
{
    if (m_project)
    {
        int i = ui->cmbDevice->currentIndex();
        return m_project->device(i);
    }
    return nullptr;
}

void mbClientSendMessageUi::setEnableParams(bool enable)
{
    ui->grCommonParams ->setEnabled(enable);
    ui->grFunctionData ->setEnabled(enable);
    ui->lsList         ->setEnabled(enable);
    ui->btnListInsert  ->setEnabled(enable);
    ui->btnListEdit    ->setEnabled(enable);
    ui->btnListImport  ->setEnabled(enable);
    ui->btnListMoveUp  ->setEnabled(enable);
    ui->btnListMoveDown->setEnabled(enable);
    ui->btnListRemove  ->setEnabled(enable);
    ui->btnListClear   ->setEnabled(enable);
    ui->spPeriod       ->setEnabled(enable);
    ui->chbUseLoop     ->setEnabled(enable);
    ui->btnSendOne     ->setEnabled(enable);
    ui->btnSendList    ->setEnabled(enable);
}

void mbClientSendMessageUi::setSendTo(int type)
{
    if (m_sendTo != type)
    {
        switch (type)
        {
        case SendToPortUnit:
            m_sendTo = SendToPortUnit;
            ui->lbSendTo->setText(QStringLiteral("Port:"));
            ui->swSendTo->setCurrentWidget(ui->pgPortUnit);
            ui->rdPortUnit->setChecked(true);
            break;
        case SendToDevice:
        default:
            m_sendTo = SendToDevice;
            ui->lbSendTo->setText(QStringLiteral("Device:"));
            ui->swSendTo->setCurrentWidget(ui->pgDevice);
            ui->rdDevice->setChecked(true);
            break;
        }
    }
}

bool mbClientSendMessageUi::prepareSendParams()
{
    switch (m_sendTo)
    {
    case SendToPortUnit:
        m_port = currentPort();
        if (m_port)
        {
            const mbClientDevice::Defaults &def = mbClientDevice::Defaults::instance();
            m_dataParams.maxReadCoils               = def.maxReadCoils             ;
            m_dataParams.maxReadDiscreteInputs      = def.maxReadDiscreteInputs    ;
            m_dataParams.maxReadHoldingRegisters    = def.maxReadHoldingRegisters  ;
            m_dataParams.maxReadInputRegisters      = def.maxReadInputRegisters    ;
            m_dataParams.maxWriteMultipleCoils      = def.maxWriteMultipleCoils    ;
            m_dataParams.maxWriteMultipleRegisters  = def.maxWriteMultipleRegisters;
            m_dataParams.swapBytes                  = def.swapBytes                ;
            m_dataParams.registerOrder              = def.registerOrder            ;
            m_dataParams.byteArrayFormat            = def.byteArrayFormat          ;
            m_dataParams.stringEncoding             = def.stringEncoding           ;
            m_dataParams.stringLengthType           = def.stringLengthType         ;
            m_dataParams.byteArraySeparator         = def.byteArraySeparator       ;
            m_unit = static_cast<decltype(m_unit)>(ui->spUnit->value());
        }
        else
            return false;
        break;
    default:
        m_device = currentDevice();
        if (m_device)
        {
            m_dataParams.maxReadCoils               = m_device->maxReadCoils             ();
            m_dataParams.maxReadDiscreteInputs      = m_device->maxReadDiscreteInputs    ();
            m_dataParams.maxReadHoldingRegisters    = m_device->maxReadHoldingRegisters  ();
            m_dataParams.maxReadInputRegisters      = m_device->maxReadInputRegisters    ();
            m_dataParams.maxWriteMultipleCoils      = m_device->maxWriteMultipleCoils    ();
            m_dataParams.maxWriteMultipleRegisters  = m_device->maxWriteMultipleRegisters();
            m_dataParams.swapBytes                  = m_device->swapBytes                ();
            m_dataParams.registerOrder              = m_device->registerOrder            ();
            m_dataParams.byteArrayFormat            = m_device->byteArrayFormat          ();
            m_dataParams.stringEncoding             = m_device->stringEncoding           ();
            m_dataParams.stringLengthType           = m_device->stringLengthType         ();
            m_dataParams.byteArraySeparator         = m_device->byteArraySeparator       ();
        }
        else
            return false;
        break;
    }
    return true;
}

void mbClientSendMessageUi::fillParams(mbClientMessageParams &params)
{
    params.func = getCurrentFuncNum();
    switch(params.func)
    {
    case MBF_READ_COILS:
    case MBF_READ_DISCRETE_INPUTS:
    case MBF_READ_HOLDING_REGISTERS:
    case MBF_READ_INPUT_REGISTERS:
        params.offset = getDefaultOffset();
        params.count  = static_cast<uint16_t>(ui->spDefaultCount->value());
        break;
    case MBF_WRITE_SINGLE_COIL:
    case MBF_WRITE_SINGLE_REGISTER:
        params.offset = getDefaultOffset();
        params.format = mb::enumFormatValueByIndex(ui->cmbDefaultFormat->currentIndex());
        params.data = ui->txtDefaultData->toPlainText();
        break;
    case MBF_READ_EXCEPTION_STATUS:
        break;
    case MBF_DIAGNOSTICS:
        params.subfunc = getCurrentDiagnSubfuncNum();
        params.format = mb::enumFormatValueByIndex(ui->cmbDiagnFormat->currentIndex());
        params.data = ui->txtDiagnRequest->toPlainText();
        break;
    case MBF_GET_COMM_EVENT_COUNTER:
    case MBF_GET_COMM_EVENT_LOG:
        break;
    case MBF_WRITE_MULTIPLE_COILS:
    case MBF_WRITE_MULTIPLE_REGISTERS:
        params.offset = getDefaultOffset();
        params.count  = static_cast<uint16_t>(ui->spDefaultCount->value());
        params.format = mb::enumFormatValueByIndex(ui->cmbDefaultFormat->currentIndex());
        params.data = ui->txtDefaultData->toPlainText();
        break;
    case MBF_REPORT_SERVER_ID:
        break;
    case MBF_READ_FILE_RECORD:
        break;
    case MBF_WRITE_FILE_RECORD:
        break;
    case MBF_MASK_WRITE_REGISTER:
        params.offset = getWriteMaskOffset();
        params.format = mb::Hex16; //mb::enumFormatValueByIndex(ui->cmbWriteMaskFormat->currentIndex());
        params.andMask = static_cast<uint16_t>(ui->spWriteMaskAnd->value());
        params.orMask = static_cast<uint16_t>(ui->spWriteMaskOr->value());
        break;
    case MBF_READ_WRITE_MULTIPLE_REGISTERS:
        params.offset      = getRWMultiRegReadOffset();
        params.count       = static_cast<uint16_t>(ui->spRWMultiRegReadCount   ->value());
        params.format = mb::enumFormatValueByIndex(ui->cmbRWMultiRegReadFormat->currentIndex());
        params.writeOffset = getRWMultiRegWriteOffset();
        params.writeCount  = static_cast<uint16_t>(ui->spRWMultiRegWriteCount  ->value());
        params.writeFormat = mb::enumFormatValueByIndex(ui->cmbRWMultiRegWriteFormat->currentIndex());
        params.data = ui->txtRWMultiRegWriteData->toPlainText();
        break;
    case MBF_READ_FIFO_QUEUE:
        params.offset = static_cast<uint16_t>(ui->spFIFOOffset->value());
        params.format = mb::enumFormatValueByIndex(ui->cmbFIFOFormat->currentIndex());
        break;
    case MBF_ENCAPSULATED_INTERFACE_TRANSPORT:
        params.deviceId = static_cast<uint8_t>(ui->spReadDeviceId->value());
        params.objectId = static_cast<uint8_t>(ui->spReadDeviceObjectId->value());
        params.format = mb::enumFormatValueByIndex(ui->cmbReadDeviceFormat->currentIndex());
    default:
        return;
    }
}

void mbClientSendMessageUi::fillForm(const mbClientMessageParams &params)
{
    setCurrentFuncNum(params.func);
    switch(params.func)
    {
    case MBF_READ_COILS:
    case MBF_READ_DISCRETE_INPUTS:
    case MBF_READ_HOLDING_REGISTERS:
    case MBF_READ_INPUT_REGISTERS:
        setDefaultOffset(params.offset);
        ui->cmbDefaultFormat->setCurrentText(mb::enumFormatKey(params.format));
        ui->spDefaultCount->setValue(params.count);
        break;
    case MBF_WRITE_SINGLE_COIL:
    case MBF_WRITE_SINGLE_REGISTER:
        setDefaultOffset(params.offset);
        ui->cmbDefaultFormat->setCurrentText(mb::enumFormatKey(params.format));
        ui->txtDefaultData->setPlainText(params.data);
        break;
    case MBF_READ_EXCEPTION_STATUS:
    case MBF_REPORT_SERVER_ID:
        ui->cmbReadDataFormat->setCurrentText(mb::enumFormatKey(params.format));
        break;
    case MBF_DIAGNOSTICS:
        setCurrentDiagnSubfuncNum(params.subfunc);
        ui->cmbDiagnFormat->setCurrentText(mb::enumFormatKey(params.format));
        ui->txtDiagnRequest->setPlainText(params.data);
        break;
    case MBF_WRITE_MULTIPLE_COILS:
    case MBF_WRITE_MULTIPLE_REGISTERS:
        setDefaultOffset(params.offset);
        ui->spDefaultCount->setValue(params.count);
        ui->cmbDefaultFormat->setCurrentText(mb::enumFormatKey(params.format));
        ui->txtDefaultData->setPlainText(params.data);
        break;
    case MBF_READ_FILE_RECORD:
        break;
    case MBF_WRITE_FILE_RECORD:
        break;
    case MBF_MASK_WRITE_REGISTER:
    {
        setWriteMaskOffset(params.offset);
        ui->spWriteMaskAnd->setValue(params.andMask);
        ui->spWriteMaskOr ->setValue(params.orMask );
    }
        break;
    case MBF_READ_WRITE_MULTIPLE_REGISTERS:
        setRWMultiRegReadOffset(params.offset);
        ui->spRWMultiRegReadCount->setValue(params.count);
        ui->cmbRWMultiRegReadFormat->setCurrentText(mb::enumFormatKey(params.format));
        setRWMultiRegWriteOffset(params.writeOffset);
        ui->spRWMultiRegWriteCount->setValue(params.writeCount);
        ui->cmbRWMultiRegWriteFormat->setCurrentText(mb::enumFormatKey(params.writeFormat));
        ui->txtRWMultiRegWriteData->setPlainText(params.data);
        break;
    case MBF_READ_FIFO_QUEUE:
        ui->spFIFOOffset->setValue(params.offset);
        ui->cmbFIFOFormat->setCurrentText(mb::enumFormatKey(params.format));
        break;
    case MBF_ENCAPSULATED_INTERFACE_TRANSPORT:
        ui->spReadDeviceId->setValue(params.deviceId);
        ui->spReadDeviceObjectId->setValue(params.objectId);
        break;
    default:
        return;
    }
}

void mbClientSendMessageUi::fillForm(const mbClientRunMessagePtr &message)
{
    if (!Modbus::StatusIsGood(message->status()))
        return;
    QStringList ls;
    mb::Format format = mb::enumFormatValueByIndex(ui->cmbDefaultFormat->currentIndex());
    QPlainTextEdit *txt = ui->txtDefaultData;
    switch (message->function())
    {
    case MBF_READ_COILS:
    case MBF_READ_DISCRETE_INPUTS:
    {
        uint16_t count = message->count();
        QByteArray data((count+7)/8, '\0');
        message->getData(0, count, data.data());
        switch (format)
        {
        case mb::Bool:
            ls = toStringListBits(data, count);
            break;
        case mb::ByteArray:
        case mb::String:
            ls.append(mb::toVariant(data,
                                    format,
                                    Modbus::Memory_0x,
                                    m_dataParams.swapBytes,
                                    m_dataParams.registerOrder,
                                    m_dataParams.byteArrayFormat,
                                    m_dataParams.stringEncoding,
                                    m_dataParams.stringLengthType,
                                    m_dataParams.byteArraySeparator,
                                    data.count()).toString());
            break;
        default:
            ls = toStringListNumbers(data, format);
            break;
        }
    }
        break;
    case MBF_READ_EXCEPTION_STATUS:
    {
        txt = ui->txtReadData;
        format = mb::enumFormatValueByIndex(ui->cmbReadDataFormat->currentIndex());
        uint16_t count = 8;
        QByteArray data((count+7)/8, '\0');
        message->getData(0, count, data.data());
        switch (format)
        {
        case mb::Bool:
            ls = toStringListBits(data, count);
            break;
        case mb::ByteArray:
        case mb::String:
            ls.append(mb::toVariant(data,
                                    format,
                                    Modbus::Memory_0x,
                                    m_dataParams.swapBytes,
                                    m_dataParams.registerOrder,
                                    m_dataParams.byteArrayFormat,
                                    m_dataParams.stringEncoding,
                                    m_dataParams.stringLengthType,
                                    m_dataParams.byteArraySeparator,
                                    data.count()).toString());
            break;
        default:
            ls = toStringListNumbers(data, format);
            break;
        }
    }
        break;
    case MBF_READ_WRITE_MULTIPLE_REGISTERS:
        txt = ui->txtRWMultiRegReadData;
        format = mb::enumFormatValueByIndex(ui->cmbRWMultiRegReadFormat->currentIndex());
        MB_FALLTHROUGH
    case MBF_READ_HOLDING_REGISTERS:
    case MBF_READ_INPUT_REGISTERS:
    {
        uint16_t count = message->count();
        QByteArray data(count * 2, '\0');
        message->getData(0, count, data.data());
        switch (format)
        {
        case mb::Bool:
            ls = toStringListBits(data, count * 16);
            break;
        case mb::ByteArray:
        case mb::String:
            ls.append(mb::toVariant(data,
                                    format,
                                    Modbus::Memory_0x,
                                    m_dataParams.swapBytes,
                                    m_dataParams.registerOrder,
                                    m_dataParams.byteArrayFormat,
                                    m_dataParams.stringEncoding,
                                    m_dataParams.stringLengthType,
                                    m_dataParams.byteArraySeparator,
                                    data.count()).toString());
            break;
        default:
            ls = toStringListNumbers(data, format);
            break;
        }
    }
        break;
    case MBF_DIAGNOSTICS:
    {
        txt = ui->txtDiagnResponse;
        uint16_t count = message->count();
        QByteArray data(reinterpret_cast<char*>(message->innerBuffer()), count);
        format = mb::enumFormatValueByIndex(ui->cmbDiagnFormat->currentIndex());
        switch (format)
        {
        case mb::Bool:
            ls = toStringListBits(data, count*8);
            break;
        case mb::ByteArray:
        case mb::String:
            ls.append(mb::toVariant(data,
                                    format,
                                    Modbus::Memory_0x,
                                    m_dataParams.swapBytes,
                                    m_dataParams.registerOrder,
                                    m_dataParams.byteArrayFormat,
                                    m_dataParams.stringEncoding,
                                    m_dataParams.stringLengthType,
                                    m_dataParams.byteArraySeparator,
                                    data.count()).toString());
            break;
        default:
            ls = toStringListNumbers(data, format);
            break;
        }
    }
        break;
    case MBF_GET_COMM_EVENT_COUNTER:
    {
        auto m = reinterpret_cast<mbClientRunMessageGetCommEventCounter*>(message.data());
        ui->lnGetCommEventCounterStatus->setText(mb::toHexString(m->status()));
        ui->lnGetCommEventCounterCount->setText(mb::toDecString(m->eventCount()));
    }
        break;
    case MBF_GET_COMM_EVENT_LOG:
    {
        auto m = reinterpret_cast<mbClientRunMessageGetCommEventCounter*>(message.data());
        ui->lnGetCommEventLogStatus->setText(mb::toHexString(m->status()));
        ui->lnGetCommEventLogEventCount->setText(mb::toDecString(m->eventCount()));
        ui->lnGetCommEventLogMessageCount->setText(mb::toDecString(m->eventCount()));
        ui->tblEventLog->clearContents();
        for (int i = 0; i < m->count(); i++)
        {
            uint8_t eventId = reinterpret_cast<uint8_t*>(m->innerBuffer())[i];
            ui->tblEventLog->insertRow(i);
            QTableWidgetItem *item0 = new QTableWidgetItem(mb::toHexString(eventId));
            QTableWidgetItem *item1 = new QTableWidgetItem(getEventLogDescription(eventId));
            ui->tblEventLog->setItem(i, 0, item0);
            ui->tblEventLog->setItem(i, 1, item1);
        }
    }
        break;
    case MBF_REPORT_SERVER_ID:
    {
        txt = ui->txtReadData;
        format = mb::enumFormatValueByIndex(ui->cmbReadDataFormat->currentIndex());
        uint16_t count = message->count();
        QByteArray data(count, '\0');
        message->getData(0, count, data.data());
        switch (format)
        {
        case mb::Bool:
            ls = toStringListBits(data, count);
            break;
        case mb::ByteArray:
        case mb::String:
            ls.append(mb::toVariant(data,
                                    format,
                                    Modbus::Memory_0x,
                                    m_dataParams.swapBytes,
                                    m_dataParams.registerOrder,
                                    m_dataParams.byteArrayFormat,
                                    m_dataParams.stringEncoding,
                                    m_dataParams.stringLengthType,
                                    m_dataParams.byteArraySeparator,
                                    data.count()).toString());
            break;
        default:
            ls = toStringListNumbers(data, format);
            break;
        }
    }
        break;
    case MBF_READ_FILE_RECORD:
        return;
    case MBF_WRITE_FILE_RECORD:
        return;
    case MBF_READ_FIFO_QUEUE:
    {
        uint16_t count = message->count();
        QByteArray data(count * 2, '\0');
        message->getData(0, count, data.data());
        switch (format)
        {
        case mb::Bool:
            ls = toStringListBits(data, count * 16);
            break;
        case mb::ByteArray:
        case mb::String:
            ls.append(mb::toVariant(data,
                                    format,
                                    Modbus::Memory_0x,
                                    m_dataParams.swapBytes,
                                    m_dataParams.registerOrder,
                                    m_dataParams.byteArrayFormat,
                                    m_dataParams.stringEncoding,
                                    m_dataParams.stringLengthType,
                                    m_dataParams.byteArraySeparator,
                                    data.count()).toString());
            break;
        default:
            ls = toStringListNumbers(data, format);
            break;
        }
    }
        break;
    case MBF_ENCAPSULATED_INTERFACE_TRANSPORT:
    {
        format = mb::enumFormatValueByIndex(ui->cmbReadDeviceFormat->currentIndex());
        auto m = reinterpret_cast<mbClientRunMessageReadDeviceId*>(message.data());
        ui->lnReadDeviceConformity->setText(mb::toDecString(m->conformityLevel()));
        ui->lnReadDeviceNextObjectId->setText(mb::toDecString(m->nextObjectId()));
        ui->chbReadDeviceMoreFollows->setChecked(m->moreFollows());
        ui->tblReadDeviceObjects->setRowCount(0);
        auto sz = m->dataSize();
        for (int i = 0, c = 0; i+2 < sz; ++c)
        {
            uint8_t objectId = reinterpret_cast<uint8_t*>(m->innerBuffer())[i];
            uint8_t len = reinterpret_cast<uint8_t*>(m->innerBuffer())[i+1];
            if (i + 1 + len >= sz)
                len = sz - i - 2;
            QByteArray data(reinterpret_cast<char*>(m->innerBuffer()) + i + 2, len);
            auto v = mb::toVariant(data,
                                   format,
                                   Modbus::Memory_0x,
                                   m_dataParams.swapBytes,
                                   m_dataParams.registerOrder,
                                   m_dataParams.byteArrayFormat,
                                   m_dataParams.stringEncoding,
                                   m_dataParams.stringLengthType,
                                   m_dataParams.byteArraySeparator,
                                   data.count()).toString();
            ui->tblReadDeviceObjects->insertRow(c);
            QTableWidgetItem *item0 = new QTableWidgetItem(mb::toHexString(objectId));
            QTableWidgetItem *item1 = new QTableWidgetItem(v);
            ui->tblReadDeviceObjects->setItem(c, 0, item0);
            ui->tblReadDeviceObjects->setItem(c, 1, item1);
            i += len + 2;
        }
    }
        break;
    default:
        return;
    }
    //txt->clear();
    if (ls.count())
    {
        QStringList::ConstIterator it = ls.constBegin();
        QString s = *it;
        for (++it; it != ls.constEnd(); ++it)
            s += QStringLiteral(",") + *it;
        txt->setPlainText(s);
    }
}

QStringList mbClientSendMessageUi::toStringListBits(const QByteArray &data, uint16_t count)
{
    QStringList ls;
    for (int i = 0; i < count; i++)
    {
        bool v = data.at(i / 8) & (1 << (i % 8));
        QString s = v ? QStringLiteral("1") : QStringLiteral("0");
        ls.append(s);
    }
    return ls;
}

QStringList mbClientSendMessageUi::toStringListNumbers(const QByteArray &data, mb::Format format)
{
    QStringList ls;
    int sz = static_cast<int>(mb::sizeofFormat(format));
    int c = data.size() / sz;
    for (int i = 0; i < c; i++)
    {
        QByteArray numData = data.mid(i*sz, sz);
        QString s = mb::toVariant(numData,
                                  format,
                                  Modbus::Memory_4x,
                                  m_dataParams.swapBytes,
                                  m_dataParams.registerOrder,
                                  m_dataParams.byteArrayFormat,
                                  m_dataParams.stringEncoding,
                                  m_dataParams.stringLengthType,
                                  m_dataParams.byteArraySeparator,
                                  numData.count()).toString();
        ls.append(s);
    }
    size_t szRemainder = data.size() - c * sz;
    if (szRemainder)
    {
        quint64 v = 0;
        memcpy(&v, &data.data()[c*sz], szRemainder);
        QByteArray numData(reinterpret_cast<char*>(&v), sizeof(v));
        QString s = mb::toVariant(numData,
                                  format,
                                  Modbus::Memory_4x,
                                  m_dataParams.swapBytes,
                                  m_dataParams.registerOrder,
                                  m_dataParams.byteArrayFormat,
                                  m_dataParams.stringEncoding,
                                  m_dataParams.stringLengthType,
                                  m_dataParams.byteArraySeparator,
                                  numData.count()).toString();
        ls.append(s);
    }
    return ls;
}

QByteArray mbClientSendMessageUi::fromStringListBits(const QStringList &ls)
{
    QByteArray r((ls.count()+7)/8, '\0');
    int i = 0;
    Q_FOREACH(const QString &s, ls)
    {
        if (s == QStringLiteral("1"))
            r.data()[i / 8] |= (1 << (i % 8));
        i++;
    }
    return r;
}

QByteArray mbClientSendMessageUi::fromStringListNumbers(const QStringList &ls, mb::Format format)
{
    QByteArray data;
    Q_FOREACH(const QString &s, ls)
        data.append(mb::toByteArray(s,
                                    format,
                                    Modbus::Memory_4x,
                                    m_dataParams.swapBytes,
                                    m_dataParams.registerOrder,
                                    m_dataParams.byteArrayFormat,
                                    m_dataParams.stringEncoding,
                                    m_dataParams.stringLengthType,
                                    m_dataParams.byteArraySeparator,
                                    0));

    return data;
}

bool mbClientSendMessageUi::fromStringNumber(mb::Format format, const QString &v, void *buff)
{
    bool ok = false;
    switch (format)
    {
    case mb::Bin16:
        *reinterpret_cast<quint16*>(buff) = v.toUShort(&ok, 2);
        break;
    case mb::Oct16:
        *reinterpret_cast<quint16*>(buff) = v.toUShort(&ok, 8);
        break;
    case mb::Dec16:
        *reinterpret_cast<qint16*>(buff) = v.toShort(&ok, 10);
        break;
    case mb::UDec16:
        *reinterpret_cast<quint16*>(buff) = v.toUShort(&ok, 10);
        break;
    case mb::Hex16:
        *reinterpret_cast<quint16*>(buff) = v.toUShort(&ok, 16);
        break;
    case mb::Bin32:
        *reinterpret_cast<quint32*>(buff) = v.toULong(&ok, 2);
        break;
    case mb::Oct32:
        *reinterpret_cast<quint32*>(buff) = v.toULong(&ok, 8);
        break;
    case mb::Dec32:
        *reinterpret_cast<qint32*>(buff) = v.toLong(&ok, 10);
        break;
    case mb::UDec32:
        *reinterpret_cast<quint32*>(buff) = v.toULong(&ok, 10);
        break;
    case mb::Hex32:
        *reinterpret_cast<quint32*>(buff) = v.toULong(&ok, 16);
        break;
    case mb::Bin64:
        *reinterpret_cast<quint64*>(buff) = v.toULongLong(&ok, 2);
        break;
    case mb::Oct64:
        *reinterpret_cast<quint64*>(buff) = v.toULongLong(&ok, 8);
        break;
    case mb::Dec64:
        *reinterpret_cast<qint64*>(buff) = v.toLongLong(&ok, 10);
        break;
    case mb::UDec64:
        *reinterpret_cast<quint64*>(buff) = v.toULongLong(&ok, 10);
        break;
    case mb::Hex64:
        *reinterpret_cast<quint64*>(buff) = v.toULongLong(&ok, 16);
        break;
    case mb::Float:
        *reinterpret_cast<float*>(buff) = v.toFloat(&ok);
        break;
    case mb::Double:
        *reinterpret_cast<double*>(buff) = v.toDouble(&ok);
        break;
    default:
        break;
    }
    return ok;
}

QStringList mbClientSendMessageUi::dataToStringList(const QString &s)
{
    QStringList ls = s.split(',');
    return ls;
}

uint8_t mbClientSendMessageUi::getCurrentFuncNum() const
{
    return m_funcNums.value(ui->cmbFunction->currentIndex());
}

uint16_t mbClientSendMessageUi::getCurrentDiagnSubfuncNum() const
{
    return m_diagnSubfuncNums.value(ui->cmbDiagnSubfunction->currentIndex());
}

void mbClientSendMessageUi::setCurrentFuncNum(uint8_t func)
{
    switch(func)
    {
    case MBF_READ_COILS:
        ui->swFunctionData->setCurrentWidget(ui->pgDefault);
        m_defaultAddress->setAddressType(Modbus::Memory_0x);
        m_defaultAddress->setEnabledAddress(true);
        ui->spDefaultCount->setEnabled(true);
        ui->txtDefaultData->setReadOnly(true);
        break;
    case MBF_READ_DISCRETE_INPUTS:
        ui->swFunctionData->setCurrentWidget(ui->pgDefault);
        m_defaultAddress->setAddressType(Modbus::Memory_1x);
        m_defaultAddress->setEnabledAddress(true);
        ui->spDefaultCount->setEnabled(true);
        ui->txtDefaultData->setReadOnly(true);
        break;
    case MBF_READ_HOLDING_REGISTERS:
        ui->swFunctionData->setCurrentWidget(ui->pgDefault);
        m_defaultAddress->setAddressType(Modbus::Memory_4x);
        m_defaultAddress->setEnabledAddress(true);
        ui->spDefaultCount->setEnabled(true);
        ui->txtDefaultData->setReadOnly(true);
        break;
    case MBF_READ_INPUT_REGISTERS:
        ui->swFunctionData->setCurrentWidget(ui->pgDefault);
        m_defaultAddress->setAddressType(Modbus::Memory_3x);
        m_defaultAddress->setEnabledAddress(true);
        ui->spDefaultCount->setEnabled(true);
        ui->txtDefaultData->setReadOnly(true);
        break;
    case MBF_WRITE_SINGLE_COIL:
        ui->swFunctionData->setCurrentWidget(ui->pgDefault);
        m_defaultAddress->setAddressType(Modbus::Memory_0x);
        m_defaultAddress->setEnabledAddress(true);
        ui->spDefaultCount->setEnabled(false);
        ui->txtDefaultData->setReadOnly(false);
        break;
    case MBF_WRITE_SINGLE_REGISTER:
        ui->swFunctionData->setCurrentWidget(ui->pgDefault);
        m_defaultAddress->setAddressType(Modbus::Memory_4x);
        m_defaultAddress->setEnabledAddress(true);
        ui->spDefaultCount->setEnabled(false);
        ui->txtDefaultData->setReadOnly(false);
        break;
    case MBF_READ_EXCEPTION_STATUS:
    case MBF_REPORT_SERVER_ID:
        ui->swFunctionData->setCurrentWidget(ui->pgReadData);
        break;
    case MBF_DIAGNOSTICS:
        ui->swFunctionData->setCurrentWidget(ui->pgDiagn);
        break;
    case MBF_GET_COMM_EVENT_COUNTER:
        ui->swFunctionData->setCurrentWidget(ui->pgGetCommEventCounter);
        break;
    case MBF_GET_COMM_EVENT_LOG:
        ui->swFunctionData->setCurrentWidget(ui->pgGetCommEventLog);
        break;
    case MBF_WRITE_MULTIPLE_COILS:
        ui->swFunctionData->setCurrentWidget(ui->pgDefault);
        m_defaultAddress->setAddressType(Modbus::Memory_0x);
        m_defaultAddress->setEnabledAddress(true);
        ui->spDefaultCount->setEnabled(true);
        ui->txtDefaultData->setReadOnly(false);
        break;
    case MBF_WRITE_MULTIPLE_REGISTERS:
        ui->swFunctionData->setCurrentWidget(ui->pgDefault);
        m_defaultAddress->setAddressType(Modbus::Memory_4x);
        m_defaultAddress->setEnabledAddress(true);
        ui->spDefaultCount->setEnabled(true);
        ui->txtDefaultData->setReadOnly(false);
        break;
    case MBF_READ_FILE_RECORD:
        ui->swFunctionData->setCurrentWidget(ui->pgFileRecords);
        break;
    case MBF_WRITE_FILE_RECORD:
        ui->swFunctionData->setCurrentWidget(ui->pgFileRecords);
        break;
    case MBF_MASK_WRITE_REGISTER:
        ui->swFunctionData->setCurrentWidget(ui->pgWriteMask);
        m_writeMaskAddress->setAddressType(Modbus::Memory_4x);
        break;
    case MBF_READ_WRITE_MULTIPLE_REGISTERS:
        ui->swFunctionData->setCurrentWidget(ui->pgRWMultiReg);
        m_rwMultiRegReadAddress ->setAddressType(Modbus::Memory_4x);
        m_rwMultiRegWriteAddress->setAddressType(Modbus::Memory_4x);
        break;
    case MBF_READ_FIFO_QUEUE:
        ui->swFunctionData->setCurrentWidget(ui->pgFIFO);
        break;
    case MBF_ENCAPSULATED_INTERFACE_TRANSPORT:
        ui->swFunctionData->setCurrentWidget(ui->pgReadDeviceId);
        break;
    }
    int i = 0;
    Q_FOREACH (int f, m_funcNums)
    {
        if (f == func)
        {
            ui->cmbFunction->setCurrentIndex(i);
            break;
        }
        ++i;
    }
}

void mbClientSendMessageUi::setCurrentDiagnSubfuncNum(uint16_t subfunc)
{
    switch (subfunc)
    {
    case MBF_DIAGNOSTICS_RETURN_QUERY_DATA:
    case MBF_DIAGNOSTICS_RETURN_DIAGNOSTIC_REGISTER:
    case MBF_DIAGNOSTICS_CHANGE_ASCII_INPUT_DELIMITER:
        ui->txtDiagnRequest->setEnabled(true);
        break;
    default:
        ui->txtDiagnRequest->setEnabled(false);
        break;
    }

    int i = 0;
    Q_FOREACH (int f, m_diagnSubfuncNums)
    {
        if (f == subfunc)
        {
            ui->cmbDiagnSubfunction->setCurrentIndex(i);
            break;
        }
        ++i;
    }
}

uint16_t mbClientSendMessageUi::getDefaultOffset() const
{
    return m_defaultAddress->getAddress().offset();
}

void mbClientSendMessageUi::setDefaultOffset(uint16_t v)
{
    mb::Address adr = m_defaultAddress->getAddress();
    adr.setOffset(v);
    m_defaultAddress->setAddress(adr);
}

int mbClientSendMessageUi::getDefaultAddress() const
{
    return m_defaultAddress->getAddress().number();
}

void mbClientSendMessageUi::setDefaultAddress(int v)
{
    mb::Address adr = m_defaultAddress->getAddress();
    adr.setNumber(v);
    m_defaultAddress->setAddress(adr);
}

uint16_t mbClientSendMessageUi::getRWMultiRegWriteOffset() const
{
    return m_rwMultiRegWriteAddress->getAddress().offset();
}

void mbClientSendMessageUi::setRWMultiRegWriteOffset(uint16_t v)
{
    mb::Address adr = m_rwMultiRegWriteAddress->getAddress();
    adr.setOffset(v);
    m_rwMultiRegWriteAddress->setAddress(adr);
}

int mbClientSendMessageUi::getRWMultiRegWriteAddress() const
{
    return m_rwMultiRegWriteAddress->getAddress().number();
}

void mbClientSendMessageUi::setRWMultiRegWriteAddress(int v)
{
    mb::Address adr = m_rwMultiRegWriteAddress->getAddress();
    adr.setNumber(v);
    m_rwMultiRegWriteAddress->setAddress(adr);
}

uint16_t mbClientSendMessageUi::getRWMultiRegReadOffset() const
{
    return m_rwMultiRegReadAddress->getAddress().offset();
}

void mbClientSendMessageUi::setRWMultiRegReadOffset(uint16_t v)
{
    mb::Address adr = m_rwMultiRegReadAddress->getAddress();
    adr.setOffset(v);
    m_rwMultiRegReadAddress->setAddress(adr);
}

int mbClientSendMessageUi::getRWMultiRegReadAddress() const
{
    return m_rwMultiRegReadAddress->getAddress().number();
}

void mbClientSendMessageUi::setRWMultiRegReadAddress(int v)
{
    mb::Address adr = m_rwMultiRegReadAddress->getAddress();
    adr.setNumber(v);
    m_rwMultiRegReadAddress->setAddress(adr);
}

uint16_t mbClientSendMessageUi::getWriteMaskOffset() const
{
    return m_writeMaskAddress->getAddress().offset();
}

void mbClientSendMessageUi::setWriteMaskOffset(uint16_t v)
{
    mb::Address adr = m_writeMaskAddress->getAddress();
    adr.setOffset(v);
    m_writeMaskAddress->setAddress(adr);
}

int mbClientSendMessageUi::getWriteMaskAddress() const
{
    return m_writeMaskAddress->getAddress().number();
}

void mbClientSendMessageUi::setWriteMaskAddress(int v)
{
    mb::Address adr = m_writeMaskAddress->getAddress();
    adr.setNumber(v);
    m_writeMaskAddress->setAddress(adr);
}

