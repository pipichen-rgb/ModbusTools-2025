#include "client_dialogscannerrequest.h"
#include "ui_client_dialogscannerrequest.h"

#include <mbcore.h>

mbClientDialogScannerRequest::Strings::Strings() :
    prefix        (QStringLiteral("Ui.Scanner.Dialogs.Request.")),
    func          (prefix+QStringLiteral("func")),
    subfunc       (prefix+QStringLiteral("subfunc")),
    offset1       (prefix+QStringLiteral("offset1")),
    offset2       (prefix+QStringLiteral("offset2")),
    count2        (prefix+QStringLiteral("count2")),
    wGeometry     (prefix+QStringLiteral("geometry")),
    wSplitterState(prefix+QStringLiteral("splitterState"))
{
}

const mbClientDialogScannerRequest::Strings &mbClientDialogScannerRequest::Strings::instance()
{
    static Strings s;
    return s;
}

mbClientDialogScannerRequest::mbClientDialogScannerRequest(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::mbClientDialogScannerRequest)
{
    ui->setupUi(this);

    m_model = new Model(this);

    ui->lsRequest->setModel(m_model);
    ui->lsRequest->setSelectionMode(QAbstractItemView::SingleSelection);

    QSpinBox *sp;
    QComboBox *cmb;

    sp = ui->spOffset1;
    sp->setMinimum(0);
    sp->setMaximum(USHRT_MAX);
    sp->setValue(0);

    sp = ui->spOffset2;
    sp->setMinimum(0);
    sp->setMaximum(USHRT_MAX);
    sp->setValue(0);

    sp = ui->spCount2;
    sp->setMinimum(0);
    sp->setMaximum(USHRT_MAX);
    sp->setValue(1);

    m_funcNums.append(MBF_READ_COILS                   );
    m_funcNums.append(MBF_READ_DISCRETE_INPUTS         );
    m_funcNums.append(MBF_READ_HOLDING_REGISTERS       );
    m_funcNums.append(MBF_READ_INPUT_REGISTERS         );
    m_funcNums.append(MBF_WRITE_SINGLE_COIL            );
    m_funcNums.append(MBF_WRITE_SINGLE_REGISTER        );
    m_funcNums.append(MBF_READ_EXCEPTION_STATUS        );
    m_funcNums.append(MBF_DIAGNOSTICS                  );
    m_funcNums.append(MBF_GET_COMM_EVENT_COUNTER       );
    m_funcNums.append(MBF_GET_COMM_EVENT_LOG           );
    m_funcNums.append(MBF_WRITE_MULTIPLE_COILS         );
    m_funcNums.append(MBF_WRITE_MULTIPLE_REGISTERS     );
    m_funcNums.append(MBF_REPORT_SERVER_ID             );
    m_funcNums.append(MBF_MASK_WRITE_REGISTER          );
    m_funcNums.append(MBF_READ_WRITE_MULTIPLE_REGISTERS);
    m_funcNums.append(MBF_READ_FIFO_QUEUE              );

    m_diagnSubfuncNums.append(MBDIAGN_RETURN_QUERY_DATA                     );
    m_diagnSubfuncNums.append(MBDIAGN_RESTART_COMMUNICATIONS_OPTION         );
    m_diagnSubfuncNums.append(MBDIAGN_RETURN_DIAGNOSTIC_REGISTER            );
    m_diagnSubfuncNums.append(MBDIAGN_CHANGE_ASCII_INPUT_DELIMITER          );
    m_diagnSubfuncNums.append(MBDIAGN_FORCE_LISTEN_ONLY_MODE                );
    m_diagnSubfuncNums.append(MBDIAGN_CLEAR_COUNTERS_AND_DIAGNOSTIC_REGISTER);
    m_diagnSubfuncNums.append(MBDIAGN_RETURN_BUS_MESSAGE_COUNT              );
    m_diagnSubfuncNums.append(MBDIAGN_RETURN_BUS_COMMUNICATION_ERROR_COUNT  );
    m_diagnSubfuncNums.append(MBDIAGN_RETURN_BUS_EXCEPTION_ERROR_COUNT      );
    m_diagnSubfuncNums.append(MBDIAGN_RETURN_SERVER_MESSAGE_COUNT           );
    m_diagnSubfuncNums.append(MBDIAGN_RETURN_SERVER_NO_RESPONSE_COUNT       );
    m_diagnSubfuncNums.append(MBDIAGN_RETURN_SERVER_NAK_COUNT               );
    m_diagnSubfuncNums.append(MBDIAGN_RETURN_SERVER_BUSY_COUNT              );
    m_diagnSubfuncNums.append(MBDIAGN_RETURN_BUS_CHARACTER_OVERRUN_COUNT    );
    m_diagnSubfuncNums.append(MBDIAGN_CLEAR_OVERRUN_COUNTER_AND_FLAG        );

    cmb = ui->cmbFunction;
    connect(cmb, SIGNAL(currentIndexChanged(int)), this, SLOT(setCurrentFuncIndex(int)));
    Q_FOREACH (uint8_t funcNum, m_funcNums)
    {
        cmb->addItem(QString("%1 - %2")
                    .arg(funcNum, 2, 10, QChar('0'))
                    .arg(mb::ModbusFunctionString(funcNum))
                    );
    }
    cmb->setCurrentIndex(2);

    cmb = ui->cmbDiagnSubfunction;
    //connect(cmb, SIGNAL(currentIndexChanged(int)), this, SLOT(setCurrentDiagnSubfuncNum(int)));
    Q_FOREACH (auto funcNum, m_diagnSubfuncNums)
    {
        cmb->addItem(QString("%1 - %2")
                         .arg(funcNum, 2, 10, QChar('0'))
                         .arg(mb::ModbusDiagnSubfunctionString(funcNum))
                     );
    }
    cmb->setCurrentIndex(0);

    connect(ui->lsRequest->selectionModel(), &QItemSelectionModel::currentRowChanged, this, &mbClientDialogScannerRequest::selectionChanged);
    connect(ui->btnAdd   , &QPushButton::clicked, this, &mbClientDialogScannerRequest::addFunc   );
    connect(ui->btnModify, &QPushButton::clicked, this, &mbClientDialogScannerRequest::modifyFunc);
    connect(ui->btnDelete, &QPushButton::clicked, this, &mbClientDialogScannerRequest::deleteFunc);

}

mbClientDialogScannerRequest::~mbClientDialogScannerRequest()
{
    delete ui;
}

MBSETTINGS mbClientDialogScannerRequest::cachedSettings() const
{
    MBSETTINGS m;
    const Strings &s = Strings::instance();

    m[s.func          ] = getCurrentFuncNum();
    m[s.subfunc       ] = getCurrentDiagnSubfuncNum();
    m[s.offset1       ] = ui->spOffset1        ->value      ();
    m[s.offset2       ] = ui->spOffset2        ->value      ();
    m[s.count2        ] = ui->spCount2         ->value      ();
    m[s.wGeometry     ] = this->saveGeometry();
    m[s.wSplitterState] = ui->splitter->saveState();

    return m;
}

void mbClientDialogScannerRequest::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();
    //bool ok;

    it = m.find(s.func          ); if (it != end) setCurrentFuncNum(static_cast<uint8_t>(it.value().toUInt()));
    it = m.find(s.subfunc       ); if (it != end) setCurrentDiagnSubfuncNum(static_cast<uint16_t>(it.value().toUInt()));
    it = m.find(s.offset1       ); if (it != end) ui->spOffset1        ->setValue       (it.value().toInt()   );
    it = m.find(s.offset2       ); if (it != end) ui->spOffset2        ->setValue       (it.value().toInt()   );
    it = m.find(s.count2        ); if (it != end) ui->spCount2         ->setValue       (it.value().toInt()   );
    it = m.find(s.wGeometry     ); if (it != end) this                 ->restoreGeometry(it.value().toByteArray());
    it = m.find(s.wSplitterState); if (it != end) ui->splitter         ->restoreState   (it.value().toByteArray());
}

bool mbClientDialogScannerRequest::getRequest(mbClientScanner::Request_t &req)
{
    m_model->setRequest(req);
    if (req.count())
        ui->lsRequest->selectionModel()->select(m_model->index(0), QItemSelectionModel::Select);
    switch (QDialog::exec())
    {
    case QDialog::Accepted:
        req = m_model->request();
        return true;
    }
    return false;
}

void mbClientDialogScannerRequest::selectionChanged(const QModelIndex &current, const QModelIndex &)
{
    mbClientMessageParams func = m_model->func(current.row());
    setCurrentFunc(func);
}

void mbClientDialogScannerRequest::addFunc()
{
    mbClientMessageParams func = getCurrentFunc();
    m_model->addFunc(func);
}

void mbClientDialogScannerRequest::modifyFunc()
{
    QModelIndexList ls = ui->lsRequest->selectionModel()->selectedRows();
    if (ls.count())
    {
        mbClientMessageParams func = getCurrentFunc();
        m_model->modifyFunc(ls.first().row(), func);
    }
}

void mbClientDialogScannerRequest::deleteFunc()
{
    QModelIndexList ls = ui->lsRequest->selectionModel()->selectedRows();
    if (ls.count())
    {
        m_model->deleteFunc(ls.first().row());
    }
}

void mbClientDialogScannerRequest::setCurrentFuncIndex(int i)
{
    uint8_t funcNum = m_funcNums.value(i);
    setCurrentFuncNum(funcNum);
}

mbClientMessageParams mbClientDialogScannerRequest::getCurrentFunc() const
{
    uint8_t funcNum = getCurrentFuncNum();
    mbClientMessageParams func;
    switch(funcNum)
    {
    case MBF_READ_COILS:
    case MBF_READ_DISCRETE_INPUTS:
    case MBF_READ_HOLDING_REGISTERS:
    case MBF_READ_INPUT_REGISTERS:
    case MBF_WRITE_MULTIPLE_COILS:
    case MBF_WRITE_MULTIPLE_REGISTERS:
    case MBF_READ_WRITE_MULTIPLE_REGISTERS:
        func.func   = funcNum;
        func.offset = static_cast<uint16_t>(ui->spOffset2->value());
        func.count  = static_cast<uint16_t>(ui->spCount2 ->value());
        break;
    case MBF_WRITE_SINGLE_COIL:
    case MBF_WRITE_SINGLE_REGISTER:
    case MBF_MASK_WRITE_REGISTER:
    case MBF_READ_FIFO_QUEUE:
        func.func   = funcNum;
        func.offset = static_cast<uint16_t>(ui->spOffset1->value());
        func.count  = 1;
        break;
    case MBF_READ_EXCEPTION_STATUS:
    case MBF_REPORT_SERVER_ID:
    case MBF_GET_COMM_EVENT_COUNTER:
    case MBF_GET_COMM_EVENT_LOG:
        func.func   = funcNum;
        break;
    case MBF_DIAGNOSTICS:
        func.func   = funcNum;
        func.subfunc= getCurrentDiagnSubfuncNum();
        break;
    }
    return func;
}

void mbClientDialogScannerRequest::setCurrentFunc(const mbClientMessageParams &f)
{
    int index = m_funcNums.indexOf(f.func);
    if (index < 0)
        return;
    switch(f.func)
    {
    case MBF_READ_COILS:
    case MBF_READ_DISCRETE_INPUTS:
    case MBF_READ_HOLDING_REGISTERS:
    case MBF_READ_INPUT_REGISTERS:
    case MBF_WRITE_MULTIPLE_COILS:
    case MBF_WRITE_MULTIPLE_REGISTERS:
    case MBF_READ_WRITE_MULTIPLE_REGISTERS:
        ui->spOffset2->setValue(f.offset);
        ui->spCount2 ->setValue(f.count);
        break;
    case MBF_WRITE_SINGLE_COIL:
    case MBF_WRITE_SINGLE_REGISTER:
    case MBF_MASK_WRITE_REGISTER:
    case MBF_READ_FIFO_QUEUE:
        ui->spOffset1->setValue(f.offset);
        break;
    case MBF_READ_EXCEPTION_STATUS:
    case MBF_REPORT_SERVER_ID:
    case MBF_GET_COMM_EVENT_COUNTER:
    case MBF_GET_COMM_EVENT_LOG:
        break;
    case MBF_DIAGNOSTICS:
        setCurrentDiagnSubfuncNum(f.subfunc);
        break;
    default:
        return;
    }
    ui->cmbFunction->setCurrentIndex(index);
}

uint8_t mbClientDialogScannerRequest::getCurrentFuncNum() const
{
    return m_funcNums.value(ui->cmbFunction->currentIndex());
}

uint16_t mbClientDialogScannerRequest::getCurrentDiagnSubfuncNum() const
{
    return m_diagnSubfuncNums.value(ui->cmbDiagnSubfunction->currentIndex());
}

void mbClientDialogScannerRequest::setCurrentFuncNum(uint8_t funcNum)
{
    if (getCurrentFuncNum() != funcNum)
    {
        int index = m_funcNums.indexOf(funcNum);
        if (index < 0)
            return;
        ui->cmbFunction->setCurrentIndex(index);
    }
    switch(funcNum)
    {
    case MBF_READ_COILS:
    case MBF_READ_DISCRETE_INPUTS:
    case MBF_READ_HOLDING_REGISTERS:
    case MBF_READ_INPUT_REGISTERS:
    case MBF_WRITE_MULTIPLE_COILS:
    case MBF_WRITE_MULTIPLE_REGISTERS:
    case MBF_READ_WRITE_MULTIPLE_REGISTERS:
        ui->swFuncParams->setCurrentWidget(ui->pgOffsetCount);
        break;
    case MBF_WRITE_SINGLE_COIL:
    case MBF_WRITE_SINGLE_REGISTER:
    case MBF_MASK_WRITE_REGISTER:
    case MBF_READ_FIFO_QUEUE:
        ui->swFuncParams->setCurrentWidget(ui->pgOffset);
        break;
    case MBF_DIAGNOSTICS:
        ui->swFuncParams->setCurrentWidget(ui->pgDiagn);
        break;
    default:
        ui->swFuncParams->setCurrentWidget(ui->pgEmpty);
        break;
    }
}

void mbClientDialogScannerRequest::setCurrentDiagnSubfuncNum(uint16_t subfunc)
{
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

/* =============================================================
 * =========================== MODEL ===========================
 */
int mbClientDialogScannerRequest::Model::rowCount(const QModelIndex &) const
{
    return m_req.count();
}

QVariant mbClientDialogScannerRequest::Model::data(const QModelIndex &index, int role) const
{
    if (role == Qt::DisplayRole)
        return mbClientScanner::toString(m_req.value(index.row()));
    return QVariant();
}

void mbClientDialogScannerRequest::Model::setRequest(const mbClientScanner::Request_t &req)
{
    beginResetModel();
    m_req = req;
    endResetModel();
}

mbClientMessageParams mbClientDialogScannerRequest::Model::func(int i)
{
    return m_req.value(i);
}

void mbClientDialogScannerRequest::Model::addFunc(const mbClientMessageParams &f)
{
    int c = m_req.count();
    beginInsertRows(QModelIndex(), c, c);
    m_req.append(f);
    endInsertRows();
}

void mbClientDialogScannerRequest::Model::modifyFunc(int i, const mbClientMessageParams &f)
{
    if ((0 <= i) && (i < m_req.size()))
    {
        m_req[i] = f;
        QModelIndex ind = index(i);
        Q_EMIT dataChanged(ind, ind);
    }
}

void mbClientDialogScannerRequest::Model::deleteFunc(int i)
{
    if ((0 <= i) && (i < m_req.size()))
    {
        beginRemoveRows(QModelIndex(), i, i);
        m_req.removeAt(i);
        endRemoveRows();
    }
}
