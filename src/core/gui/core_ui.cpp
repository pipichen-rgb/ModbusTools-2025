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
#include "core_ui.h"

#include <QCloseEvent>
#include <QLabel>
#include <QDockWidget>
#include <QStatusBar>
#include <QApplication>
#include <QClipboard>
#include <QBuffer>
#include <QAction>
#include <QMenu>

#include <core.h>

#include <project/core_dom.h>
#include <project/core_project.h>
#include <project/core_port.h>
#include <project/core_device.h>
#include <project/core_dataview.h>
#include <project/core_builder.h>

#include "dialogs/core_dialogs.h"
#include "dialogs/core_dialogdataviewitem.h"

#include "project/core_projectui.h"
#include "dataview/core_dataviewmanager.h"
#include "dataview/core_dataviewui.h"
#include "help/core_helpui.h"

#include "core_windowmanager.h"
#include "logview/core_logview.h"

#define RECENT_PROJECTS_COUNT 20

mbCoreUi::Strings::Strings() :
    settings_useNameWithSettings(QStringLiteral("Ui.useNameWithSettings")),
    settings_recentProjects(QStringLiteral("Ui.recentProjects")),
    wGeometry(QStringLiteral("Ui.geometry")),
    wState(QStringLiteral("Ui.windowState"))
{
}

const mbCoreUi::Strings &mbCoreUi::Strings::instance()
{
    static const Strings s;
    return s;
}

mbCoreUi::Defaults::Defaults() :
    settings_useNameWithSettings(true)
{
}

const mbCoreUi::Defaults &mbCoreUi::Defaults::instance()
{
    static const Defaults d;
    return d;
}

mbCoreUi::mbCoreUi(mbCore *core, QWidget *parent) :
    QMainWindow(parent),
    m_core(core)
{
    m_project = nullptr;
    connect(core, &mbCore::projectChanged, this, &mbCoreUi::setProject);

    m_logView = new mbCoreLogView(this);
    m_builder = m_core->builderCore();
    m_dialogs = nullptr;
    m_windowManager = nullptr;
    m_dataViewManager = nullptr;
    m_currentPort = nullptr;
    m_projectUi = nullptr;
    m_tray = nullptr;
    m_help = nullptr;

    m_menuRecent = new QMenu(this);
    m_actionFileRecentClear = new QAction("Clear", m_menuRecent);
    m_menuRecent->addAction(m_actionFileRecentClear);
    connect(m_menuRecent, &QMenu::triggered, this, &mbCoreUi::menuRecentTriggered);
}

mbCoreUi::~mbCoreUi()
{
    delete m_dialogs;
}

QWidget *mbCoreUi::logView() const
{
    return m_logView;
}

void mbCoreUi::initialize()
{
    m_ui.dockLogView->setWidget(logView());

    m_help = new mbCoreHelpUi(m_helpFile, this);

    connect(m_projectUi, &mbCoreProjectUi::portDoubleClick   , this, &mbCoreUi::menuSlotPortEdit  );
    connect(m_projectUi, &mbCoreProjectUi::portContextMenu   , this, &mbCoreUi::contextMenuPort   );
    connect(m_projectUi, &mbCoreProjectUi::currentPortChanged, this, &mbCoreUi::currentPortChanged);
    m_ui.dockProject->setWidget(m_projectUi);

    connect(m_dataViewManager, &mbCoreDataViewManager::dataViewUiContextMenu, this, &mbCoreUi::contextMenuDataViewUi);
    connect(m_dataViewManager, &mbCoreDataViewManager::dataViewUiAdd        , this, &mbCoreUi::dataViewWindowAdd    );
    connect(m_dataViewManager, &mbCoreDataViewManager::dataViewUiRemove     , this, &mbCoreUi::dataViewWindowRemove );

    m_ui.actionWindowViewSubWindow->setCheckable(true);
    m_ui.actionWindowViewTabbed->setCheckable(true);
    slotWindowManagerViewModeChanged(m_windowManager->viewMode());
    connect(m_windowManager, &mbCoreWindowManager::viewModeChanged, this, &mbCoreUi::slotWindowManagerViewModeChanged);
    this->setCentralWidget(m_windowManager->centralWidget());

    // Menu File
    m_ui.actionFileRecent->setMenu(m_menuRecent);
    m_ui.actionFileNew   ->setShortcuts(QKeySequence::New   );
    m_ui.actionFileOpen  ->setShortcuts(QKeySequence::Open  );
    m_ui.actionFileSave  ->setShortcuts(QKeySequence::Save  );
    m_ui.actionFileSaveAs->setShortcuts(QKeySequence::SaveAs);
    m_ui.actionFileEdit  ->setShortcut (QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_E));
    m_ui.actionFileQuit  ->setShortcuts(QKeySequence::Quit);

    connect(m_ui.actionFileNew          , &QAction::triggered, this, &mbCoreUi::menuSlotFileNew          );
    connect(m_ui.actionFileOpen         , &QAction::triggered, this, &mbCoreUi::menuSlotFileOpen         );
    connect(m_ui.actionFileClose        , &QAction::triggered, this, &mbCoreUi::menuSlotFileClose        );
    connect(m_ui.actionFileSave         , &QAction::triggered, this, &mbCoreUi::menuSlotFileSave         );
    connect(m_ui.actionFileSaveAs       , &QAction::triggered, this, &mbCoreUi::menuSlotFileSaveAs       );
    connect(m_ui.actionFileEdit         , &QAction::triggered, this, &mbCoreUi::menuSlotFileEdit         );
    connect(m_ui.actionFileImportProject, &QAction::triggered, this, &mbCoreUi::menuSlotFileImportProject);
    connect(m_ui.actionFileInfo         , &QAction::triggered, this, &mbCoreUi::menuSlotFileInfo         );
    connect(m_ui.actionFileQuit         , &QAction::triggered, this, &mbCoreUi::menuSlotFileQuit         );

    // Menu Edit
    //m_ui.actionEditUndo     ->setShortcuts(QKeySequence::Undo                );
    //m_ui.actionEditRedo     ->setShortcuts(QKeySequence::Redo                );
    m_ui.actionEditCut      ->setShortcuts(QKeySequence::Cut                 );
    m_ui.actionEditCopy     ->setShortcuts(QKeySequence::Copy                );
    m_ui.actionEditPaste    ->setShortcuts(QKeySequence::Paste               );
    m_ui.actionEditInsert   ->setShortcut (QKeySequence(Qt::Key_Insert      ));
    m_ui.actionEditEdit     ->setShortcut (QKeySequence(Qt::CTRL | Qt::Key_E));
    m_ui.actionEditDelete   ->setShortcuts(QKeySequence::Delete              );
    m_ui.actionEditSelectAll->setShortcuts(QKeySequence::SelectAll           );

    //connect(m_ui.actionEditUndo      , &QAction::triggered, this, &mbCoreUi::menuSlotEditUndo     );
    //connect(m_ui.actionEditRedo      , &QAction::triggered, this, &mbCoreUi::menuSlotEditRedo     );
    connect(m_ui.actionEditCut       , &QAction::triggered, this, &mbCoreUi::menuSlotEditCut      );
    connect(m_ui.actionEditCopy      , &QAction::triggered, this, &mbCoreUi::menuSlotEditCopy     );
    connect(m_ui.actionEditPaste     , &QAction::triggered, this, &mbCoreUi::menuSlotEditPaste    );
    connect(m_ui.actionEditInsert    , &QAction::triggered, this, &mbCoreUi::menuSlotEditInsert   );
    connect(m_ui.actionEditEdit      , &QAction::triggered, this, &mbCoreUi::menuSlotEditEdit     );
    connect(m_ui.actionEditDelete    , &QAction::triggered, this, &mbCoreUi::menuSlotEditDelete   );
    connect(m_ui.actionEditSelectAll , &QAction::triggered, this, &mbCoreUi::menuSlotEditSelectAll);

    // Menu View
    connect(m_ui.actionViewProject, &QAction::triggered, this, &mbCoreUi::menuSlotViewProject);
    connect(m_ui.actionViewLogView, &QAction::triggered, this, &mbCoreUi::menuSlotViewLogView);

    // Menu Port
    m_ui.actionPortNew->setShortcut(QKeySequence(Qt::ALT | Qt::Key_N));

    connect(m_ui.actionPortNew            , &QAction::triggered, this, &mbCoreUi::menuSlotPortNew           );
    connect(m_ui.actionPortEdit           , &QAction::triggered, this, &mbCoreUi::menuSlotPortEdit          );
    connect(m_ui.actionPortDelete         , &QAction::triggered, this, &mbCoreUi::menuSlotPortDelete        );
    connect(m_ui.actionPortImport         , &QAction::triggered, this, &mbCoreUi::menuSlotPortImport        );
    connect(m_ui.actionPortExport         , &QAction::triggered, this, &mbCoreUi::menuSlotPortExport        );

    // Menu Device
    m_ui.actionDeviceNew->setShortcut(QKeySequence(Qt::SHIFT | Qt::Key_N));

    connect(m_ui.actionDeviceNew   , &QAction::triggered, this, &mbCoreUi::menuSlotDeviceNew   );
    connect(m_ui.actionDeviceEdit  , &QAction::triggered, this, &mbCoreUi::menuSlotDeviceEdit  );
    connect(m_ui.actionDeviceDelete, &QAction::triggered, this, &mbCoreUi::menuSlotDeviceDelete);
    connect(m_ui.actionDeviceImport, &QAction::triggered, this, &mbCoreUi::menuSlotDeviceImport);
    connect(m_ui.actionDeviceExport, &QAction::triggered, this, &mbCoreUi::menuSlotDeviceExport);

    // Menu DataView
    connect(m_ui.actionDataViewItemNew    , &QAction::triggered, this, &mbCoreUi::menuSlotDataViewItemNew    );
    connect(m_ui.actionDataViewItemEdit   , &QAction::triggered, this, &mbCoreUi::menuSlotDataViewItemEdit   );
    connect(m_ui.actionDataViewItemInsert , &QAction::triggered, this, &mbCoreUi::menuSlotDataViewItemInsert );
    connect(m_ui.actionDataViewItemDelete , &QAction::triggered, this, &mbCoreUi::menuSlotDataViewItemDelete );
    connect(m_ui.actionDataViewImportItems, &QAction::triggered, this, &mbCoreUi::menuSlotDataViewImportItems);
    connect(m_ui.actionDataViewExportItems, &QAction::triggered, this, &mbCoreUi::menuSlotDataViewExportItems);
    connect(m_ui.actionDataViewNew        , &QAction::triggered, this, &mbCoreUi::menuSlotDataViewNew        );
    connect(m_ui.actionDataViewEdit       , &QAction::triggered, this, &mbCoreUi::menuSlotDataViewEdit       );
    connect(m_ui.actionDataViewInsert     , &QAction::triggered, this, &mbCoreUi::menuSlotDataViewInsert     );
    connect(m_ui.actionDataViewDelete     , &QAction::triggered, this, &mbCoreUi::menuSlotDataViewDelete     );
    connect(m_ui.actionDataViewImport     , &QAction::triggered, this, &mbCoreUi::menuSlotDataViewImport     );
    connect(m_ui.actionDataViewExport     , &QAction::triggered, this, &mbCoreUi::menuSlotDataViewExport     );

    // Menu Tools
    connect(m_ui.actionToolsSettings   , &QAction::triggered, this, &mbCoreUi::menuSlotToolsSettings   );

    // Menu Runtime
    m_ui.actionRuntimeStartStop->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_R));
    connect(m_ui.actionRuntimeStartStop , &QAction::triggered, this, &mbCoreUi::menuSlotRuntimeStartStop);

    // Menu Window
    connect(m_ui.actionWindowViewSubWindow   , &QAction::triggered, this, &mbCoreUi::menuSlotWindowViewSubWindow   );
    connect(m_ui.actionWindowViewTabbed      , &QAction::triggered, this, &mbCoreUi::menuSlotWindowViewTabbed      );
    connect(m_ui.actionWindowDataViewCloseAll, &QAction::triggered, this, &mbCoreUi::menuSlotWindowDataViewCloseAll);
    connect(m_ui.actionWindowCloseAll        , &QAction::triggered, this, &mbCoreUi::menuSlotWindowCloseAll        );
    connect(m_ui.actionWindowCascade         , &QAction::triggered, this, &mbCoreUi::menuSlotWindowCascade         );
    connect(m_ui.actionWindowTile            , &QAction::triggered, this, &mbCoreUi::menuSlotWindowTile            );

    // Menu Help
    m_ui.actionHelpContents->setShortcuts(QKeySequence::HelpContents);
    connect(m_ui.actionHelpAbout   , &QAction::triggered, this, &mbCoreUi::menuSlotHelpAbout   );
    connect(m_ui.actionHelpAboutQt , &QAction::triggered, this, &mbCoreUi::menuSlotHelpAboutQt );
    connect(m_ui.actionHelpContents, &QAction::triggered, this, &mbCoreUi::menuSlotHelpContents);

    // status bar
    m_lbSystemStatus = new QLabel(m_ui.statusbar);
    m_lbSystemStatus->setFrameShape(QFrame::Panel);
    m_lbSystemStatus->setFrameStyle(QFrame::Sunken);
    m_lbSystemStatus->setAutoFillBackground(true);
    m_lbSystemStatus->setMinimumWidth(100);

    m_lbPortName = new QLabel("Port", m_ui.statusbar);
    m_lbPortName->setFrameShape(QFrame::Panel);
    m_lbPortName->setFrameStyle(QFrame::Sunken);
    m_lbPortName->setAutoFillBackground(true);
    m_lbPortName->setMinimumWidth(100);

    m_lbPortStatTx = new QLabel("0", m_ui.statusbar);
    m_lbPortStatTx->setFrameShape(QFrame::Panel);
    m_lbPortStatTx->setFrameStyle(QFrame::Sunken);
    m_lbPortStatTx->setAutoFillBackground(true);
    m_lbPortStatTx->setMinimumWidth(70);

    m_lbPortStatRx = new QLabel("0", m_ui.statusbar);
    m_lbPortStatRx->setFrameShape(QFrame::Panel);
    m_lbPortStatRx->setFrameStyle(QFrame::Sunken);
    m_lbPortStatRx->setAutoFillBackground(true);
    m_lbPortStatRx->setMinimumWidth(70);

    statusChange(m_core->status());
    m_ui.statusbar->addPermanentWidget(m_lbPortName);
    m_ui.statusbar->addPermanentWidget(new QLabel("Tx: ", m_ui.statusbar));
    m_ui.statusbar->addPermanentWidget(m_lbPortStatTx);
    m_ui.statusbar->addPermanentWidget(new QLabel("Rx: ", m_ui.statusbar));
    m_ui.statusbar->addPermanentWidget(m_lbPortStatRx);
    m_ui.statusbar->addPermanentWidget(new QLabel("Status: ", m_ui.statusbar));
    m_ui.statusbar->addPermanentWidget(m_lbSystemStatus, 1);
    refreshCurrentPortName();

    connect(m_core, SIGNAL(statusChanged(int)), this, SLOT(statusChange(int)));

    const bool tray = false;
    if (m_core->args().value(mbCore::Arg_Tray, tray).toBool())
    {
        m_tray = new QSystemTrayIcon(this);
        QMenu* menu = new QMenu(this);
        QAction* actionShow = new QAction("Show", menu);
        connect(actionShow, SIGNAL(triggered()), this, SLOT(show()));
        QAction* actionQuit = new QAction("Quit", menu);
        connect(actionShow, SIGNAL(triggered()), this, SLOT(fileQuit()));
        connect(m_tray, SIGNAL(activated(QSystemTrayIcon::ActivationReason)),
                this, SLOT(slotTrayActivated(QSystemTrayIcon::ActivationReason)));
        menu->addAction(actionShow);
        menu->addSeparator();
        menu->addAction(actionQuit);
        m_tray->setContextMenu(menu);
        m_tray->setToolTip(m_core->applicationName());
        m_tray->setIcon(this->windowIcon());
        m_tray->show();
        qApp->setQuitOnLastWindowClosed(false);
    }
}

bool mbCoreUi::useNameWithSettings() const
{
    return m_projectUi->useNameWithSettings();
}

void mbCoreUi::setUseNameWithSettings(bool use)
{
    m_projectUi->setUseNameWithSettings(use);
    refreshCurrentPortName();
}

MBSETTINGS mbCoreUi::cachedSettings() const
{
    const Strings &s = Strings::instance();
    MBSETTINGS r = m_dialogs->cachedSettings();
    mb::unite(r, m_logView->cachedSettings());
    mb::unite(r, m_help->cachedSettings());
    r[s.settings_useNameWithSettings] = useNameWithSettings();
    r[s.settings_recentProjects] = cachedSettingsRecentProjects();
    r[s.wGeometry] = this->saveGeometry();
    r[s.wState   ] = this->saveState();

    return r;
}

void mbCoreUi::setCachedSettings(const MBSETTINGS &settings)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = settings.end();
    //bool ok;

    it = settings.find(s.settings_useNameWithSettings);
    if (it != end)
    {
        bool v = it.value().toBool();
        setUseNameWithSettings(v);
    }

    it = settings.find(s.settings_recentProjects);
    if (it != end)
    {
        QVariantList v = it.value().toList();
        setCachedSettingsRecentProjects(v);
    }

    it = settings.find(s.wGeometry);
    if (it != end)
    {
        this->restoreGeometry(it.value().toByteArray());
    }

    it = settings.find(s.wState);
    if (it != end)
    {
        this->restoreState(it.value().toByteArray());
    }

    m_dialogs->setCachedSettings(settings);
    m_logView->setCachedSettings(settings);
    m_help->setCachedSettings(settings);
}

void mbCoreUi::logMessage(mb::LogFlag flag, const QString &source, const QString &text)
{
    m_logView->logMessage(flag, source, text);
}

void mbCoreUi::outputMessage(const QString &/*message*/)
{
}

void mbCoreUi::menuSlotFileNew()
{
    if (m_core->isRunning())
        return;
    MBSETTINGS settings = m_dialogs->getProject(MBSETTINGS(), QStringLiteral("New Project"));
    if (settings.count())
    {
        mbCoreProject* p = m_builder->newProject();
        p->setSettings(settings);
        p->setModifiedFlag(true);
        m_core->setProjectCore(p);
    }
}

void mbCoreUi::menuSlotFileOpen()
{
    if (m_core->isRunning())
        return;
    checkProjectModifiedAndSave(QStringLiteral("Open Project"), QStringLiteral("open another one"));
    QString file = m_dialogs->getOpenFileName(this,
                                              QStringLiteral("Open Project..."),
                                              QString(),
                                              m_dialogs->getFilterString(mbCoreDialogs::Filter_ProjectAll));
    if (!file.isEmpty())
    {
        closeProject();
        openProject(file);
    }
}

void mbCoreUi::menuSlotFileClose()
{
    if (m_core->isRunning())
        return;
    QMessageBox::StandardButton res = checkProjectModifiedAndSave(QStringLiteral("Close Project"),
                                                                  QStringLiteral("close"),
                                                                  QMessageBox::Yes|QMessageBox::No|QMessageBox::Cancel);
    if (res != QMessageBox::Cancel)
        closeProject();
}

void mbCoreUi::menuSlotFileSave()
{
    if (m_project)
    {
        if (m_project->absoluteFilePath().isEmpty())
        {
            menuSlotFileSaveAs();
            return;
        }
        saveProjectInner();
        m_project->setWindowsData(m_windowManager->saveWindowsState());
        m_project->resetVersion();
        if (m_core->builderCore()->saveCore(m_project))
            addRecentFile(m_project->absoluteFilePath());
    }
}

void mbCoreUi::menuSlotFileSaveAs()
{
    if (m_project)
    {
        QString dir = m_project->absoluteDirPath();
        QString file = m_dialogs->getSaveFileName(this,
                                                  QStringLiteral("Save Project..."),
                                                  dir,
                                                  m_dialogs->getFilterString(mbCoreDialogs::Filter_ProjectAll));
        if (file.isEmpty())
            return;
        m_project->setAbsoluteFilePath(file);
        menuSlotFileSave();
    }
}

void mbCoreUi::menuSlotFileEdit()
{
    if (m_core->isRunning())
        return;
    if (m_project)
    {
        MBSETTINGS old = m_project->settings();
        MBSETTINGS cur = m_dialogs->getProject(old);

        if (cur.count())
        {
            m_project->setSettings(cur);
            m_project->setModifiedFlag(true);
            setWindowModified(true);
        }
    }
}

void mbCoreUi::menuSlotFileImportProject()
{
    if (m_core->isRunning())
        return;
    checkProjectModifiedAndSave(QStringLiteral("Import Project"), QStringLiteral("import project"));
    QString file = m_dialogs->getOpenFileName(this,
                                              QStringLiteral("Import Project..."),
                                              QString(),
                                              m_dialogs->getFilterString(mbCoreDialogs::Filter_ProjectAll));
    if (!file.isEmpty())
    {
        //m_core->builderCore()->importProject(file);
        mbCoreBuilder *builder = m_core->builderCore();
        QScopedPointer<mbCoreDomProject> dom(builder->newDomProject());
        if (builder->loadXml(file, dom.data()))
        {
            int applyToAll;

            applyToAll = -1;
            Q_FOREACH(mbCoreDomDevice *d, dom->devices())
            {
                bool needToAdd = true;
                if (m_project->hasDevice(d->name()))
                {
                    int r;
                    if (applyToAll < 0)
                    {
                        r = m_dialogs->replace(QStringLiteral("Import Project"),
                                               QStringLiteral("Device '")+d->name()+QStringLiteral("' already exists."),
                                               true);
                        if (r <= 0) // Note: Cancel is clicked
                            return; // Note: it might skip additional importDomProject processing
                    }
                    else
                        r = applyToAll;
                    switch (r)
                    {
                    case mbCoreDialogReplace::ReplaceAll:
                        applyToAll = r;
                        // no need break
                    case mbCoreDialogReplace::Replace:
                    {
                        mbCoreDevice *old = m_project->deviceCore(d->name());
                        builder->fillDevice(old, d);
                        needToAdd = false;
                    }
                        break;
                    case mbCoreDialogReplace::RenameAll:
                        applyToAll = r;
                        // no need break
                    case mbCoreDialogReplace::Rename:
                        needToAdd = true;
                        break;
                    case mbCoreDialogReplace::SkipAll:
                        applyToAll = r;
                        // no need break
                    default:
                        continue;
                    }
                }
                if (needToAdd)
                {
                    mbCoreDevice *v = builder->toDevice(d);
                    m_project->deviceAdd(v);
                }
            }

            applyToAll = -1;
            Q_FOREACH(mbCoreDomPort *d, dom->ports())
            {
                bool needToAdd = true;
                if (m_project->hasPort(d->name()))
                {
                    int r;
                    if (applyToAll < 0)
                    {
                        r = m_dialogs->replace(QStringLiteral("Import Project"),
                                               QStringLiteral("Port '")+d->name()+QStringLiteral("' already exists."),
                                               true);
                        if (r <= 0) // Note: Cancel is clicked
                            return; // Note: it might skip additional importDomProject processing
                    }
                    else
                        r = applyToAll;
                    switch (r)
                    {
                    case mbCoreDialogReplace::ReplaceAll:
                        applyToAll = r;
                        // no need break
                    case mbCoreDialogReplace::Replace:
                    {
                        mbCorePort *old = m_project->portCore(d->name());
                        builder->fillPort(old, d);
                        needToAdd = false;
                    }
                        break;
                    case mbCoreDialogReplace::RenameAll:
                        applyToAll = r;
                        // no need break
                    case mbCoreDialogReplace::Rename:
                        needToAdd = true;
                        break;
                    case mbCoreDialogReplace::SkipAll:
                        applyToAll = r;
                        // no need break
                    default:
                        continue;
                    }
                }
                if (needToAdd)
                {
                    mbCorePort *v = builder->toPort(d);
                    m_project->portAdd(v);
                }
            }

            applyToAll = -1;
            Q_FOREACH(mbCoreDomDataView *d, dom->dataViews())
            {
                bool needToAdd = true;
                if (m_project->hasDataView(d->name()))
                {
                    int r;
                    if (applyToAll < 0)
                    {
                        r = m_dialogs->replace(QStringLiteral("Import Project"),
                                               QStringLiteral("DataView '")+d->name()+QStringLiteral("' already exists."),
                                               true);
                        if (r <= 0) // Note: Cancel is clicked
                            return; // Note: it might skip additional importDomProject processing
                    }
                    else
                        r = applyToAll;
                    switch (r)
                    {
                    case mbCoreDialogReplace::ReplaceAll:
                        applyToAll = r;
                        // no need break
                    case mbCoreDialogReplace::Replace:
                    {
                        mbCoreDataView *old = m_project->dataViewCore(d->name());
                        builder->fillDataView(old, d);
                        needToAdd = false;
                    }
                        break;
                    case mbCoreDialogReplace::RenameAll:
                        applyToAll = r;
                        // no need break
                    case mbCoreDialogReplace::Rename:
                        needToAdd = true;
                        break;
                    case mbCoreDialogReplace::SkipAll:
                        applyToAll = r;
                        // no need break
                    default:
                        continue;
                    }
                }
                if (needToAdd)
                {
                    mbCoreDataView *v = builder->toDataView(d);
                    m_project->dataViewAdd(v);
                }
            }
            importDomProject(dom.data());
            projectCore()->setModifiedFlag(true);
        }
    }
}

void mbCoreUi::menuSlotFileInfo()
{
    if (m_project)
        m_dialogs->showProjectInfo(m_project);
}

void mbCoreUi::menuSlotFileQuit()
{
    //m_core->application()->quit();
    close();
}

void mbCoreUi::menuSlotEditUndo()
{
    
}

void mbCoreUi::menuSlotEditRedo()
{
    
}

void mbCoreUi::menuSlotEditCut()
{
    menuSlotEditCopy();
    menuSlotEditDelete();
}

void mbCoreUi::menuSlotEditCopy()
{
    slotDataViewItemCopy();
}

void mbCoreUi::menuSlotEditPaste()
{
    slotDataViewItemPaste();
}

void mbCoreUi::menuSlotEditInsert()
{
    menuSlotDataViewItemInsert();
}

void mbCoreUi::menuSlotEditEdit()
{
    menuSlotDataViewItemEdit();
}

void mbCoreUi::menuSlotEditDelete()
{
    menuSlotDataViewItemDelete();
}

void mbCoreUi::menuSlotEditSelectAll()
{
    slotDataViewItemSelectAll();
}

void mbCoreUi::menuSlotViewProject()
{
    m_ui.dockProject->show();
}

void mbCoreUi::menuSlotViewLogView()
{
    m_ui.dockLogView->show();
}

void mbCoreUi::menuSlotPortNew()
{
}

void mbCoreUi::menuSlotPortEdit()
{
}

void mbCoreUi::menuSlotPortDelete()
{
}

void mbCoreUi::menuSlotPortImport()
{
    if (m_core->isRunning())
        return;
    if (!m_project)
        return;
    QString file = m_dialogs->getOpenFileName(this,
                                              QStringLiteral("Import Port ..."),
                                              QString(),
                                              m_dialogs->getFilterString(mbCoreDialogs::Filter_PortAll));
    if (!file.isEmpty())
    {
        if (mbCorePort *port = m_builder->importPort(file))
        {
            m_project->portAdd(port);
            m_project->setModifiedFlag(true);
        }
    }
}

void mbCoreUi::menuSlotPortExport()
{
    if (mbCorePort *port = m_projectUi->currentPortCore())
    {
        QString file = m_dialogs->getSaveFileName(this,
                                                  QString("Export Port '%1'").arg(port->name()),
                                                  QString(),
                                                  m_dialogs->getFilterString(mbCoreDialogs::Filter_PortAll));
        if (!file.isEmpty())
            m_builder->exportPort(file, port);
    }
}

void mbCoreUi::menuSlotDeviceNew()
{
}

void mbCoreUi::menuSlotDeviceEdit()
{
}

void mbCoreUi::menuSlotDeviceDelete()
{
}

void mbCoreUi::menuSlotDeviceImport()
{
}

void mbCoreUi::menuSlotDeviceExport()
{
}

void mbCoreUi::menuSlotDataViewItemNew()
{
    MBSETTINGS ns = getDataViewItemCreateSettings();
    MBSETTINGS p = dialogsCore()->getDataViewItem(ns, "New Item(s)");
    if (p.count())
    {
        const mbCoreDataViewItem::Strings &sItem = mbCoreDataViewItem::Strings::instance();
        int count = p.value(mbCoreDialogDataViewItem::Strings::instance().count).toInt();
        if (count > 0)
        {
            mbCoreDataView *wl = m_dataViewManager->activeDataViewCore();
            if (!wl)
            {
                QList<mbCoreDataViewUi*> ls = m_dataViewManager->dataViewUisCore();
                if (ls.count())
                    wl = ls.first()->dataViewCore();
                else
                {
                    if (!m_project)
                        return;
                    wl = m_builder->newDataView();
                    m_project->dataViewAdd(wl);
                }

            }
            for (int i = 0; i < count; i++)
            {
                mbCoreDataViewItem *item = builderCore()->newDataViewItem();
                item->setSettings(p);
                wl->itemAdd(item);
                p[sItem.address] = item->addressInt() + item->length();
            }
            m_project->setModifiedFlag(true);
        }
    }
}

void mbCoreUi::menuSlotDataViewItemEdit()
{
    mbCoreDataViewUi *ui = m_dataViewManager->activeDataViewUiCore();
    if (ui)
    {
        QList<mbCoreDataViewItem*> items = ui->selectedItemsCore();
        if (!items.count())
            return;
        MBSETTINGS s = items.first()->settings();
        s[mbCoreDialogDataViewItem::Strings::instance().count] = items.count();
        MBSETTINGS p = dialogsCore()->getDataViewItem(s, "Edit Item(s)");
        if (p.count())
        {
            const mbCoreDataViewItem::Strings &sItem = mbCoreDataViewItem::Strings::instance();
            Q_FOREACH (mbCoreDataViewItem *item, items)
            {
                item->setSettings(p);
                p[sItem.address] = item->addressInt() + item->length();
            }
            m_project->setModifiedFlag(true);
        }
    }
}

void mbCoreUi::menuSlotDataViewItemInsert()
{
    mbCoreDataViewUi *ui = m_dataViewManager->activeDataViewUiCore();
    if (ui)
    {
        mbCoreDataView *dataView = ui->dataViewCore();
        int index = ui->currentItemIndex();
        mbCoreDataViewItem* next = dataView->itemCore(index);
        mbCoreDataViewItem* prev;
        if (next)
            prev = dataView->itemCore(index-1);
        else
            prev = dataView->itemCore(dataView->itemCount()-1);
        mbCoreDataViewItem* newItem;
        if (prev)
        {
            newItem = m_builder->newDataViewItem(prev);
            next = dataView->itemCore(index);
        }
        else
        {
            newItem = m_builder->newDataViewItem();
            newItem->setDeviceCore(m_core->projectCore()->deviceCore(0));
        }
        dataView->itemInsert(newItem, index);
        if (next)
            ui->selectItem(next);
        m_project->setModifiedFlag(true);
    }
}

void mbCoreUi::menuSlotDataViewItemDelete()
{
    mbCoreDataViewUi *ui = m_dataViewManager->activeDataViewUiCore();
    if (ui)
    {
        QList<mbCoreDataViewItem*> items = ui->selectedItemsCore();
        if (items.count())
        {
            mbCoreDataView *wl = ui->dataViewCore();
            Q_FOREACH (mbCoreDataViewItem *item, items)
            {
                wl->itemRemove(item);
                delete item;
            }
            m_project->setModifiedFlag(true);
        }
    }
}

void mbCoreUi::menuSlotDataViewImportItems()
{
    mbCoreDataViewUi *ui = m_dataViewManager->activeDataViewUiCore();
    if (ui)
    {
        QString file = m_dialogs->getOpenFileName(this,
                                                  QStringLiteral("Import Items ..."),
                                                  QString(),
                                                  m_dialogs->getFilterString(mbCoreDialogs::Filter_DataViewItemsAll));
        if (!file.isEmpty())
        {
            auto items = m_builder->importDataViewItems(file);
            if (items.count())
            {
                mbCoreDataView *dataView = ui->dataViewCore();
                int index = ui->currentItemIndex();
                dataView->itemsInsert(items, index);
                m_project->setModifiedFlag(true);
            }
        }
    }
}

void mbCoreUi::menuSlotDataViewExportItems()
{
    mbCoreDataViewUi *ui = m_dataViewManager->activeDataViewUiCore();
    if (ui)
    {
        auto items = ui->selectedItemsCore();
        if (items.isEmpty())
            items = ui->dataViewCore()->itemsCore();
        if (items.count())
        {
            QString file = m_dialogs->getSaveFileName(this,
                                                      QStringLiteral("Export Items ..."),
                                                      QString(),
                                                      m_dialogs->getFilterString(mbCoreDialogs::Filter_DataViewItemsAll));
            if (!file.isEmpty())
                m_builder->exportDataViewItems(file, items);
        }
    }
}

void mbCoreUi::menuSlotDataViewNew()
{
    if (!m_project)
        return;
    MBSETTINGS s = dialogsCore()->getDataView(MBSETTINGS(), "New Data View");
    if (s.count())
    {
        mbCoreDataView *wl = m_builder->newDataView();
        wl->setSettings(s);
        m_project->dataViewAdd(wl);
        m_project->setModifiedFlag(true);
    }
}

void mbCoreUi::menuSlotDataViewEdit()
{
    mbCoreDataView *wl = m_dataViewManager->activeDataViewCore();
    if (wl)
    {
        MBSETTINGS p = dialogsCore()->getDataView(wl->settings(), "Edit Data View");
        if (p.count())
        {
            wl->setSettings(p);
            m_project->setModifiedFlag(true);
        }
    }
}

void mbCoreUi::menuSlotDataViewInsert()
{
    if (!m_project)
        return;
    mbCoreDataView *wl = m_builder->newDataView();
    m_project->dataViewAdd(wl);
    m_project->setModifiedFlag(true);
}

void mbCoreUi::menuSlotDataViewDelete()
{
    if (!m_project)
        return;
    mbCoreDataView *d = m_dataViewManager->activeDataViewCore();
    if (!d)
        return;
    m_project->dataViewRemove(d);
    delete d;
    m_project->setModifiedFlag(true);
}

void mbCoreUi::menuSlotDataViewImport()
{
    if (!m_project)
        return;
    QString file = m_dialogs->getOpenFileName(this,
                                              QStringLiteral("Import Data View ..."),
                                              QString(),
                                              m_dialogs->getFilterString(mbCoreDialogs::Filter_DataViewAll));
    if (!file.isEmpty())
    {
        if (mbCoreDataView *current = m_builder->importDataView(file))
        {
            m_project->dataViewAdd(current);
            m_project->setModifiedFlag(true);
        }
    }
}

void mbCoreUi::menuSlotDataViewExport()
{
    if (mbCoreDataView *current = m_dataViewManager->activeDataViewCore())
    {
        QString file = m_dialogs->getSaveFileName(this,
                                                  QStringLiteral("Export Data View ..."),
                                                  QString(),
                                                  m_dialogs->getFilterString(mbCoreDialogs::Filter_DataViewAll));
        if (!file.isEmpty())
            m_builder->exportDataView(file, current);
    }
}

void mbCoreUi::menuSlotToolsSettings()
{
    //if (m_core->isRunning())
    //    return;
    m_dialogs->editSystemSettings();
}

void mbCoreUi::menuSlotRuntimeStartStop()
{
    if (m_core->isRunning())
        m_core->stop();
    else
        m_core->start();
}

void mbCoreUi::menuSlotWindowViewSubWindow()
{
    m_windowManager->actionWindowViewSubWindow();
}

void mbCoreUi::menuSlotWindowViewTabbed()
{
    m_windowManager->actionWindowViewTabbed();
}

void mbCoreUi::menuSlotWindowDataViewCloseAll()
{
    m_windowManager->actionWindowDataViewCloseAll();
}

void mbCoreUi::menuSlotWindowCloseAll()
{
    m_windowManager->actionWindowCloseAll();
}

void mbCoreUi::menuSlotWindowCascade()
{
    m_windowManager->actionWindowCascade();
}

void mbCoreUi::menuSlotWindowTile()
{
    m_windowManager->actionWindowTile();
}

void mbCoreUi::menuSlotHelpAbout()
{
    QMessageBox::about(this, m_core->applicationName(),
                       QStringLiteral("Version: " MBTOOLS_VERSION_STR "\n") +
                       QStringLiteral("ModbusLib Version: ") + QString(Modbus::modbusLibVersionStr()) + QStringLiteral("\n")+
                       QStringLiteral("Developed by:\nSerhii Marchuk, Kyiv, Ukraine, 2023\nhttps://github.com/serhmarch"));
}

void mbCoreUi::menuSlotHelpAboutQt()
{
    QApplication::aboutQt();
}

void mbCoreUi::menuSlotHelpContents()
{
    m_help->show();
}

void mbCoreUi::slotDataViewItemCopy()
{
    mbCoreDataViewUi *ui = m_dataViewManager->activeDataViewUiCore();
    if (ui)
    {
        auto selectedItems = ui->selectedItemsCore();
        if (selectedItems.count())
        {
            QBuffer buff;
            buff.open(QIODevice::ReadWrite);
            m_builder->exportDataViewItemsXml(&buff, selectedItems);
            buff.seek(0);
            QByteArray b = buff.readAll();
            QApplication::clipboard()->setText(QString::fromUtf8(b));
        }
    }
}

void mbCoreUi::slotDataViewItemPaste()
{
    mbCoreDataViewUi *ui = m_dataViewManager->activeDataViewUiCore();
    if (ui)
    {
        QString text = QApplication::clipboard()->text();
        if (text.isEmpty())
            return;
        QByteArray b = text.toUtf8();
        QBuffer buff(&b);
        buff.open(QIODevice::ReadOnly);
        auto items = m_builder->importDataViewItemsXml(&buff);
        if (items.count())
        {
            mbCoreDataView *dataView = ui->dataViewCore();
            int index = -1;
            auto selectedItems = ui->selectedItemsCore();
            if (selectedItems.count())
                index = dataView->itemIndex(selectedItems.first());
            dataView->itemsInsert(items, index);
            m_project->setModifiedFlag(true);
        }
    }
}

void mbCoreUi::slotDataViewItemSelectAll()
{
    mbCoreDataViewUi *ui = m_dataViewManager->activeDataViewUiCore();
    if (ui)
        ui->selectAll();
}

void mbCoreUi::slotTrayActivated(QSystemTrayIcon::ActivationReason reason)
{
    switch(reason)
    {
    case QSystemTrayIcon::DoubleClick:
        show();
        break;
    default:
        break;
    }
}

void mbCoreUi::slotWindowManagerViewModeChanged(int viewMode)
{
    bool isTabbed = (viewMode == QMdiArea::TabbedView);
    m_ui.actionWindowViewSubWindow->setChecked(!isTabbed);
    m_ui.actionWindowViewTabbed->setChecked(isTabbed);
}

void mbCoreUi::contextMenuPort(mbCorePort *)
{
    QMenu mn(m_projectUi);
    Q_FOREACH(QAction *a, m_ui.menuPort->actions())
        mn.addAction(a);
    mn.exec(QCursor::pos());
}

void mbCoreUi::contextMenuDataViewUi(mbCoreDataViewUi *ui)
{
    // Note: be careful to delete deviceUi while his child 'QMenu' in stack
    // User can choose 'actionDeleteDevice' and program can crash
    // Solution: don't use direct 'delete deviceUi', use 'deviceUi->deleteLater'
    QMenu mn(ui);
    Q_FOREACH(QAction *a, m_ui.menuDataView->actions())
        mn.addAction(a);
    mn.exec(QCursor::pos());
}

void mbCoreUi::setProject(mbCoreProject *project)
{
    if (m_project)
        m_project->disconnect(this);
    m_project = project;
    if (m_project)
    {
        connect(m_project, &mbCoreProject::modifiedFlagChanged, this, &mbCoreUi::setWindowModified);
        connect(m_project, &mbCoreProject::nameChanged, this, &mbCoreUi::setProjectName);
        setProjectName(m_project->name());
        setWindowModified(m_project->isModified());
        QString absPath = m_project->absoluteFilePath();
        if (absPath.count())
            addRecentFile(absPath);
    }
    else
    {
        setProjectName(QString());
        setWindowModified(false);
    }
}

void mbCoreUi::setProjectName(const QString &name)
{
    QString title;
    if (m_project)
    {
        QString tName = name;
        if (tName.isEmpty())
            tName = QStringLiteral("<No-Name!>");
        title = QString("%1 - %2 [*]").arg(m_core->applicationName(), tName);
    }
    else
        title = m_core->applicationName();
    this->setWindowTitle(title);
}

void mbCoreUi::currentPortChanged(mbCorePort *port)
{
    mbCorePort *old = m_currentPort;
    if (old)
        old->disconnect(this);
    m_currentPort = port;
    refreshCurrentPortName();
    mbCorePort::Statistic stat;
    if (port)
    {
        connect(port, &mbCorePort::changed           , this, &mbCoreUi::refreshCurrentPortName);
        connect(port, &mbCorePort::statCountTxChanged, this, &mbCoreUi::setStatTx             );
        connect(port, &mbCorePort::statCountRxChanged, this, &mbCoreUi::setStatRx             );
        stat = port->statistic();
    }
    setStatTx(stat.countTx);
    setStatRx(stat.countRx);
}

void mbCoreUi::refreshCurrentPortName()
{
    if (m_currentPort)
    {
        if (useNameWithSettings())
            m_lbPortName->setText(m_currentPort->extendedName());
        else
            m_lbPortName->setText(m_currentPort->name());
    }
    else
        m_lbPortName->setText(QStringLiteral("-"));
    
}

void mbCoreUi::setStatTx(quint32 count)
{
    m_lbPortStatTx->setText(QString::number(count));
}

void mbCoreUi::setStatRx(quint32 count)
{
    m_lbPortStatRx->setText(QString::number(count));
}

void mbCoreUi::statusChange(int status)
{
    switch (status)
    {
    case mbCore::Running:
        //break; no need break
    case mbCore::Stopping:
    {
        //QPalette palette = m_lbSystemStatus->palette();
        QPalette palette = this->palette();
        palette.setColor(m_lbSystemStatus->backgroundRole(), Qt::green);
        palette.setColor(m_lbSystemStatus->foregroundRole(), Qt::black);
        m_lbSystemStatus->setPalette(palette);
        m_ui.actionRuntimeStartStop->setText("Stop");
        m_ui.actionRuntimeStartStop->setIcon(QIcon(":/core/icons/stop.png"));
    }
    break;
    case mbCore::Stopped:
    {
        //QPalette palette = m_lbSystemStatus->palette();
        QPalette palette = this->palette();
        palette.setColor(m_lbSystemStatus->backgroundRole(), Qt::lightGray);
        palette.setColor(m_lbSystemStatus->foregroundRole(), Qt::black);
        m_lbSystemStatus->setPalette(palette);
        m_ui.actionRuntimeStartStop->setText("Start");
        m_ui.actionRuntimeStartStop->setIcon(QIcon(":/core/icons/play.png"));
    }
    break;
    case mbCore::NoProject:
    {
        //QPalette palette = m_lbSystemStatus->palette();
        QPalette palette = this->palette();
        palette.setColor(m_lbSystemStatus->backgroundRole(), Qt::yellow);
        palette.setColor(m_lbSystemStatus->foregroundRole(), Qt::black);
        m_lbSystemStatus->setPalette(palette);
        m_ui.actionRuntimeStartStop->setText("Start");
        m_ui.actionRuntimeStartStop->setIcon(QIcon(":/core/icons/play.png"));
    }
    break;
    }
    m_lbSystemStatus->setText(mb::enumKeyTypeStr<mbCore::Status>(status));
}

void mbCoreUi::menuRecentTriggered(QAction *a)
{
    if (a == m_actionFileRecentClear)
    {
        recentClear();
    }
    else
    {
        if (m_core->isRunning())
            return;
        QString absPath = a->data().toString();
        checkProjectModifiedAndSave(QStringLiteral("Open Project"), QStringLiteral("open another one"));
        closeProject();
        openProject(absPath);
    }
}

void mbCoreUi::dataViewWindowAdd(mbCoreDataViewUi *ui)
{
    QAction *a = new QAction(ui->name());
    a->setData(QVariant::fromValue<void*>(ui));
    m_dataViewActions.insert(ui, a);
    m_ui.menuWindowDataViews->addAction(a);
    connect(ui, &mbCoreDataViewUi::nameChanged, this, &mbCoreUi::dataViewWindowRename);
    connect(a, &QAction::triggered, this, &mbCoreUi::dataViewWindowShow);
}

void mbCoreUi::dataViewWindowRemove(mbCoreDataViewUi *ui)
{
    const auto it = m_dataViewActions.find(ui);
    if (it != m_dataViewActions.end())
    {
        ui->disconnect();
        QAction *a = it.value();
        m_dataViewActions.erase(it);
        delete a;
    }
}

void mbCoreUi::dataViewWindowRename(const QString &name)
{
    mbCoreDataViewUi *ui = qobject_cast<mbCoreDataViewUi*>(sender());
    QAction *a = m_dataViewActions.value(ui);
    if (a)
    {
        a->setText(name);
    }
}

void mbCoreUi::dataViewWindowShow()
{
    QAction *a = qobject_cast<QAction*>(sender());
    if (a)
    {
        mbCoreDataViewUi *ui = reinterpret_cast<mbCoreDataViewUi*>(a->data().value<void*>());
        m_windowManager->showDataViewUi(ui);
    }
}

QMessageBox::StandardButton mbCoreUi::checkProjectModifiedAndSave(const QString &title, const QString &action, QMessageBox::StandardButtons buttons)
{
    if (m_project)
    {
        QMessageBox::StandardButton res = QMessageBox::No;
        if (m_project->isModified())
        {
            res = QMessageBox::question(this,
                                        title,
                                        QString("Save project '%1' before %2?").arg(m_project->name(), action),
                                        buttons);
            if (res == QMessageBox::Yes)
                menuSlotFileSave();
        }
        return res;
    }
    return QMessageBox::No;
}

void mbCoreUi::openProject(const QString &file)
{
    mbCoreProject* p = m_core->builderCore()->loadCore(file);
    if (p)
    {
        m_core->setProjectCore(p);
    }
    else
    {
        removeRecentFile(file);
    }
}

void mbCoreUi::closeProject()
{
    m_core->setProjectCore(nullptr);
}

void mbCoreUi::addRecentFile(const QString &absPath)
{
    QAction *a;
    auto it = m_recentProjectActions.find(absPath);
    if (it != m_recentProjectActions.end())
    {
        a = it.value();
        m_menuRecent->removeAction(a);
    }
    else
    {
        a = new QAction(absPath);
        a->setData(absPath);
        m_recentProjectActions.insert(absPath, a);
    }
    QList<QAction*> ls = m_menuRecent->actions();
    if (m_recentProjectActions.count() >= RECENT_PROJECTS_COUNT)
    {
        QAction *toRemove = ls.at(RECENT_PROJECTS_COUNT-1);
        removeRecentFile(toRemove->data().toString());
    }
    if (ls.count())
        m_menuRecent->insertAction(ls.first(), a);
    else
        m_menuRecent->addAction(a);
}

void mbCoreUi::removeRecentFile(const QString &absPath)
{
    auto it = m_recentProjectActions.find(absPath);
    if (it != m_recentProjectActions.end())
    {
        QAction *a = it.value();
        m_menuRecent->removeAction(a);
        delete a;
        m_recentProjectActions.erase(it);
    }
}

void mbCoreUi::recentClear()
{
    QList<QAction*> ls = m_menuRecent->actions();
    ls.removeAll(m_actionFileRecentClear);
    Q_FOREACH(const QAction *a, ls)
        removeRecentFile(a->data().toString());
}

QVariantList mbCoreUi::cachedSettingsRecentProjects() const
{
    QList<QAction*> ls = m_menuRecent->actions();
    ls.removeAll(m_actionFileRecentClear);
    QVariantList vs;
    Q_FOREACH (const QAction* a, ls)
    {
        vs.append(a->data());
    }
    return vs;
}

void mbCoreUi::setCachedSettingsRecentProjects(const QVariantList &ls)
{
    for (auto it = ls.rbegin(); it != ls.rend(); it++)
    {
        QString absPath = (*it).toString();
        addRecentFile(absPath);
    }
}

void mbCoreUi::closeEvent(QCloseEvent *e)
{
    QMessageBox::StandardButton res = checkProjectModifiedAndSave(QStringLiteral("Quit"),
                                                                  QStringLiteral("exit"),
                                                                  QMessageBox::Yes|QMessageBox::No|QMessageBox::Cancel);
    switch (res)
    {
    case QMessageBox::Cancel:
        e->ignore();
        break;
    default:
        e->accept();
        break;
    }
}

void mbCoreUi::importDomProject(mbCoreDomProject* /*dom*/)
{
    // Note: Base implementation does nothing
}

void mbCoreUi::saveProjectInner()
{
    // Note: base implementation do nothing
}

MBSETTINGS mbCoreUi::getDataViewItemCreateSettings()
{
    return MBSETTINGS();
}

