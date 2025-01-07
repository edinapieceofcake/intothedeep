package edu.edina.Libraries.Actions;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

// required to allow repeat annotations of type Controls

@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
public @interface ArrayOfControls {
    Controls[] value();
}
