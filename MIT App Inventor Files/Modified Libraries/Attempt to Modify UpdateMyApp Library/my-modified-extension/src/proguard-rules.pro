# Add any ProGuard configurations specific to this
# extension here.

-keep public class updatemyappmodified.mymodifiedextension.MyModifiedExtension {
    public *;
 }
-keeppackagenames gnu.kawa**, gnu.expr**

-optimizationpasses 4
-allowaccessmodification
-mergeinterfacesaggressively

-repackageclasses 'updatemyappmodified/mymodifiedextension/repack'
-flattenpackagehierarchy
-dontpreverify
