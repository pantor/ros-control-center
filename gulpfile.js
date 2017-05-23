const gulp = require('gulp'),
  babel = require('gulp-babel'),
  concat = require('gulp-concat'),
  eslint = require('gulp-eslint');

// Concat all js files into roscc.js
gulp.task('js', function () {
  return gulp.src(['app/app.js', 'app/**/*.js'])
    .pipe(babel({ presets: ['es2015'] }))
    .pipe(concat('roscc.js'))
    .pipe(gulp.dest('assets/js/'));
});

gulp.task('js-vendor', function () {
  return gulp.src([
    'node_modules/underscore/underscore.js',
    'node_modules/angular/angular.js',
    'node_modules/angular-route/angular-route.js',
    'node_modules/angular-local-storage/dist/angular-local-storage.js',
    'node_modules/angular-ui-bootstrap/dist/ui-bootstrap-tpls.js',
    'node_modules/eventemitter2/lib/eventemitter2.js',
  ])
    .pipe(concat('vendor.js'))
    .pipe(gulp.dest('assets/js/'));
});

// Lint javascript based on airbnb ES5 linter and angular code style guide
gulp.task('js-lint', function() {
  return gulp.src(['app/**/*.js'])
    .pipe(eslint()) // use .eslintrc.json file for rules
    .pipe(eslint.format())
    .pipe(eslint.failAfterError());
});

// Changes will be detected automatically
gulp.task('watch', function () {
  gulp.watch('app/**/*.js', ['js']);
});

gulp.task('default', ['js-lint', 'js-vendor', 'js']);
