package updatemyappmodified.mymodifiedextension;

import android.R;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.pm.ApplicationInfo;
import android.content.pm.PackageInfo;
import android.content.pm.PackageManager;
import android.webkit.ConsoleMessage;
import android.webkit.WebChromeClient;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.Toast;
import com.google.appinventor.components.annotations.DesignerComponent;
import com.google.appinventor.components.annotations.SimpleEvent;
import com.google.appinventor.components.annotations.SimpleFunction;
import com.google.appinventor.components.annotations.SimpleObject;
import com.google.appinventor.components.common.ComponentCategory;
import com.google.appinventor.components.runtime.ActivityStarter;
import com.google.appinventor.components.runtime.AndroidNonvisibleComponent;
import com.google.appinventor.components.runtime.ComponentContainer;
import com.google.appinventor.components.runtime.EventDispatcher;

/* JADX INFO: loaded from: C:\Users\Pribusin\Documents\classes.jar.src\classes.dex */
@DesignerComponent(category = ComponentCategory.EXTENSION, description = "Update My App extension<br><br><a href='https://drive.google.com/file/d/1SVwbBSXEgN7GaFsyIk7p2xFOJCUm_FiM/view?usp=sharing' target='pramabTAB'>Download</a>&emsp;&emsp;<a href='https://community.kodular.io/t/update-my-app-extension-useful-to-app-builders-to-update-their-distributed-apps/72417' target='pramanTAB'>How to use ?</a><br><br>It is created by Hema Srinivas, India.", helpUrl = "https://community.kodular.io/t/update-my-app-extension-useful-to-app-builders-to-update-their-distributed-apps/72417", iconName = "https://drive.google.com/uc?export=view&id=1kagAFBfLfBe_XF4s9_4KKkTG1KFbVzqa", nonVisible = true, version = 1)
@SimpleObject(external = true)
public class UpdateMyApp extends AndroidNonvisibleComponent {
    private static Context context;
    String appName;
    private ComponentContainer container;
    String dialogMessage;
    String dialogNegativeButtonText;
    String dialogPositiveButtonText;
    int versionCode;
    String versionName;

    public UpdateMyApp(ComponentContainer container) {
        super(container.$form());
        this.dialogMessage = "A new version is available...";
        this.dialogPositiveButtonText = "Update";
        this.dialogNegativeButtonText = "Cancel";
        this.container = container;
        context = container.$context();
    }

    @SimpleFunction(description = "Customizes Update Dialog even in your favourite language...")
    public void CustomizeUpdateDialog(String message, String positiveButtonText, String negativeButtonText) {
        this.dialogMessage = message;
        this.dialogPositiveButtonText = positiveButtonText;
        this.dialogNegativeButtonText = negativeButtonText;
    }

    @SimpleFunction(description = "Checks for the availability of new version of this app.")
    public void CheckForUpdate(String sheetId, String sheetName) {
        ApplicationInfo applicationInfo = context.getApplicationInfo();
        if (applicationInfo.labelRes == 0) {
            this.appName = applicationInfo.nonLocalizedLabel.toString();
        } else {
            this.appName = context.getString(applicationInfo.labelRes);
        }
        try {
            PackageInfo pInfo = context.getPackageManager().getPackageInfo(context.getPackageName(), 0);
            this.versionName = pInfo.versionName;
            this.versionCode = pInfo.versionCode;
        } catch (PackageManager.NameNotFoundException e) {
            e.printStackTrace();
            this.versionName = "";
            this.versionCode = 0;
        }
        String webAddress = "https://script.google.com/macros/s/AKfycbxUcwg6HkedZsku-VKoa3oWDatjlSgE0NojpCRG7WqtbnqTTZc/exec?sheetid=" + sheetId;
        GetDataFromWebPage((webAddress + "&sheetname=" + sheetName) + "&cellvalue=" + this.appName);
    }

    public void GetDataFromWebPage(String url) {
        WebView myWebView = new WebView(context);
        myWebView.setWebViewClient(new MyBrowser());
        myWebView.setWebChromeClient(new MyChromeBrowser());
        myWebView.getSettings().setJavaScriptEnabled(true);
        myWebView.loadUrl(url);
    }

    private class MyBrowser extends WebViewClient {
        private MyBrowser() {
        }

        @Override // android.webkit.WebViewClient
        public void onPageFinished(WebView view, String address) {
            view.loadUrl("javascript:console.log(document.getElementsByTagName('body')[0].innerHTML);");
        }
    }

    private class MyChromeBrowser extends WebChromeClient {
        private MyChromeBrowser() {
        }

        @Override // android.webkit.WebChromeClient
        public boolean onConsoleMessage(ConsoleMessage cmsg) {
            UpdateMyApp.this.AfterGettingDataFromWeb(cmsg.message());
            return true;
        }
    }

    public void AfterGettingDataFromWeb(String webData) {
        int start = webData.indexOf("&lt;pramanDIV&gt;");
        int finish = webData.indexOf("&lt;/pramanDIV&gt;");
        int error = webData.indexOf("error");
        if (start > -1 && finish > -1) {
            String webData2 = webData.substring(start + 17, finish);
            if (error == -1) {
                String[] appDetails = webData2.split(";");
                webData2 = "already updated to the latest version";
                if (Integer.parseInt(appDetails[2]) > this.versionCode) {
                    webData2 = "latest version is available";
                    DisplayMessageBox(appDetails[1]);
                }
            }
            AfterCheckingForUpdate(webData2);
        }
    }

    public void DisplayMessageBox(final String appUpdateUrl) {
        AlertDialog.Builder alertDialogBuilder = new AlertDialog.Builder(context);
        alertDialogBuilder.setIcon(R.drawable.ic_dialog_info);
        alertDialogBuilder.setTitle(this.appName);
        alertDialogBuilder.setMessage(this.dialogMessage);
        alertDialogBuilder.setPositiveButton(this.dialogPositiveButtonText, new DialogInterface.OnClickListener() { // from class: io.UpdateMyApp.UpdateMyApp.1
            @Override // android.content.DialogInterface.OnClickListener
            public void onClick(DialogInterface buttonName, int buttonId) {
                ActivityStarter myActivityStarter = new ActivityStarter(UpdateMyApp.this.container);
                myActivityStarter.Action("android.intent.action.VIEW");
                myActivityStarter.DataUri(appUpdateUrl);
                myActivityStarter.StartActivity();
                UpdateMyApp.this.InfoBox("Updating the app : " + UpdateMyApp.this.appName + ", Please wait...");
            }
        });
        alertDialogBuilder.setNegativeButton(this.dialogNegativeButtonText, new DialogInterface.OnClickListener() { // from class: io.UpdateMyApp.UpdateMyApp.2
            @Override // android.content.DialogInterface.OnClickListener
            public void onClick(DialogInterface buttonName, int buttonId) {
            }
        });
        AlertDialog alertDialog = alertDialogBuilder.create();
        alertDialog.show();
    }

    public void InfoBox(String Message) {
        Toast toast = Toast.makeText(context, Message, 1);
        toast.setGravity(17, 0, 0);
        toast.show();
    }

    @SimpleEvent(description = "After checking for the availability of new version of this app...")
    public void AfterCheckingForUpdate(String response) {
        EventDispatcher.dispatchEvent(this, "AfterCheckingForUpdate", new Object[]{response});
    }
}
