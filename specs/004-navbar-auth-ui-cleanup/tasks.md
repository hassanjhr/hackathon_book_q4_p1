# Implementation Tasks: Navbar & Auth UI Cleanup (Phase 4)

**Feature**: 004-navbar-auth-ui-cleanup
**Branch**: `004-navbar-auth-ui-cleanup`
**Generated**: 2025-12-17
**Total Tasks**: 19

---

## Overview

This document organizes implementation tasks by user story to enable independent, incremental delivery. Each phase represents a complete, testable user story.

**Tech Stack:**
- TypeScript 5.x
- React 18.x
- Docusaurus 3.9.2
- CSS Modules + Infima variables

**Working Directory:** `/my-website/`

---

## Task Format

```
- [ ] [TaskID] [P?] [Story?] Description with file path
```

- **[TaskID]**: Sequential number (T001, T002, T003...)
- **[P]**: Optional - Task is parallelizable (different files, no dependencies)
- **[Story]**: User story label ([US1], [US2], [US3], [US4]) - omitted for setup/polish phases
- **Description**: Clear action with exact file path

---

## Phase 1: Setup & Verification

**Goal**: Verify project structure and create required directories

**Independent Test Criteria:**
- [ ] Directory structure exists as specified
- [ ] No build errors after setup
- [ ] Current navbar renders correctly (baseline)

**Tasks:**

- [ ] T001 Verify working directory is /my-website/
- [ ] T002 Create component directory at my-website/src/components/NavbarItems/
- [ ] T003 Create theme directory at my-website/src/theme/NavbarItem/
- [ ] T004 Verify existing pages directory at my-website/src/pages/
- [ ] T005 Run baseline build test with `npm run build` to establish working state
- [ ] T006 Document baseline navbar items for comparison (Module 2, 3, 4 present)

**Verification:**
```bash
cd my-website
npm run build  # Must succeed before proceeding
```

---

## Phase 2: User Story 1 - Simplified Navbar

**User Story:**
> **As a** visitor
> **I want** a clean navbar without repeated module links
> **So that** navigation is simpler and I can find content via sidebar

**Acceptance Criteria:**
- [ ] Top navbar shows only Logo + "Learning" dropdown on left
- [ ] Module 2, 3, 4 links removed from navbar
- [ ] All modules still accessible via sidebar
- [ ] Build passes without errors

**Independent Test Criteria:**
- [ ] Navbar renders with only Logo + "Learning" on left side
- [ ] No Module 2, 3, 4 links visible in navbar
- [ ] Sidebar navigation still shows all modules (0-5)
- [ ] `npm run build` succeeds
- [ ] `npm run start` shows updated navbar

**Tasks:**

- [ ] T007 [US1] Open my-website/docusaurus.config.ts for editing
- [ ] T008 [US1] Locate navbar.items array in themeConfig
- [ ] T009 [US1] Remove Module 2 navbar item: `{to: '/docs/module-2/intro', label: 'Module 2', position: 'left'}`
- [ ] T010 [US1] Remove Module 3 navbar item: `{to: '/docs/module-3/intro', label: 'Module 3', position: 'left'}`
- [ ] T011 [US1] Remove Module 4 navbar item: `{to: '/docs/module-4/intro', label: 'Module 4', position: 'left'}`
- [ ] T012 [US1] Save docusaurus.config.ts with changes
- [ ] T013 [US1] Run `npm run build` to verify configuration is valid
- [ ] T014 [US1] Run `npm run start` and visually verify navbar shows only Logo + "Learning"
- [ ] T015 [US1] Test sidebar navigation - verify all modules still accessible

**Files Modified:**
- `my-website/docusaurus.config.ts`

**Verification Command:**
```bash
cd my-website
npm run build && npm run start
# Verify navbar visually in browser
```

**Story Status:** ✅ Complete and independently testable

---

## Phase 3: User Story 2 - Auth Entry Point (User Icon)

**User Story:**
> **As a** visitor
> **I want** a single user icon to access login/signup
> **So that** I don't see multiple auth options cluttering the UI

**Acceptance Criteria:**
- [ ] User icon (👤) appears in top right navbar
- [ ] Icon positioned after language/GitHub/theme toggle
- [ ] Clicking icon navigates to `/auth`
- [ ] Icon is theme-aware (light/dark mode)
- [ ] Mobile responsive (icon shows in mobile menu)

**Independent Test Criteria:**
- [ ] User icon visible in navbar (desktop)
- [ ] User icon visible in mobile menu (<996px)
- [ ] Clicking icon navigates to `/auth` (will 404 until US2 auth page built)
- [ ] Icon colors match theme (light/dark mode)
- [ ] `npm run build` succeeds

**Tasks:**

### 3.1: UserIcon Component

- [ ] T016 [P] [US2] Create my-website/src/components/NavbarItems/UserIconNavbarItem.tsx
- [ ] T017 [P] [US2] Import React and Link from '@docusaurus/Link' in UserIconNavbarItem.tsx
- [ ] T018 [P] [US2] Define UserIconNavbarItemProps interface with mobile?: boolean, authUrl?: string
- [ ] T019 [P] [US2] Create functional component UserIconNavbarItem with props destructuring
- [ ] T020 [P] [US2] Add SVG user icon (Material Design style) inside Link component
- [ ] T021 [P] [US2] Set Link to={authUrl || '/auth'} with className="navbar__link"
- [ ] T022 [P] [US2] Add ARIA label aria-label="User menu" to Link for accessibility
- [ ] T023 [P] [US2] Add conditional rendering: hide label text if mobile prop is true
- [ ] T024 [P] [US2] Export UserIconNavbarItem as default export

### 3.2: UserIcon Styles

- [ ] T025 [P] [US2] Create my-website/src/components/NavbarItems/UserIconNavbarItem.module.css
- [ ] T026 [P] [US2] Define .userIconLink class with color: var(--ifm-navbar-link-color)
- [ ] T027 [P] [US2] Add hover state with color: var(--ifm-navbar-link-hover-color)
- [ ] T028 [P] [US2] Add mobile breakpoint @media (max-width: 996px) with full-width padding
- [ ] T029 [P] [US2] Import styles in UserIconNavbarItem.tsx

### 3.3: Component Registration

- [ ] T030 [US2] Create my-website/src/theme/NavbarItem/ComponentTypes.tsx
- [ ] T031 [US2] Import ComponentTypes from '@theme-original/NavbarItem/ComponentTypes'
- [ ] T032 [US2] Import UserIconNavbarItem from '@site/src/components/NavbarItems/UserIconNavbarItem'
- [ ] T033 [US2] Export default object spreading ComponentTypes with 'custom-userIcon': UserIconNavbarItem
- [ ] T034 [US2] Verify swizzling pattern follows Docusaurus conventions

### 3.4: Navbar Configuration

- [ ] T035 [US2] Open my-website/docusaurus.config.ts for editing
- [ ] T036 [US2] Add user icon to navbar.items array after theme toggle
- [ ] T037 [US2] Configure: `{type: 'custom-userIcon', position: 'right', authUrl: '/auth'}`
- [ ] T038 [US2] Save configuration file

### 3.5: Verification

- [ ] T039 [US2] Run `npm run build` to verify no TypeScript errors
- [ ] T040 [US2] Run `npm run start` and verify user icon appears in navbar
- [ ] T041 [US2] Test user icon click (will navigate to /auth - 404 expected until US2 page built)
- [ ] T042 [US2] Test mobile responsive: resize to <996px, verify icon in menu
- [ ] T043 [US2] Test theme toggle: switch light/dark, verify icon colors change
- [ ] T044 [US2] Test keyboard navigation: Tab to icon, Enter to activate

**Files Created:**
- `my-website/src/components/NavbarItems/UserIconNavbarItem.tsx`
- `my-website/src/components/NavbarItems/UserIconNavbarItem.module.css`
- `my-website/src/theme/NavbarItem/ComponentTypes.tsx`

**Files Modified:**
- `my-website/docusaurus.config.ts`

**Verification Commands:**
```bash
cd my-website
npm run build
npm run start
# Test in browser: click user icon, test responsive, test theme
```

**Story Status:** ✅ Complete and independently testable (auth page 404 is expected)

---

## Phase 4: User Story 2 - Auth Choice Page

**User Story:**
> **As a** visitor
> **I want** a single user icon to access login/signup
> **So that** I don't see multiple auth options cluttering the UI

**Continuation:** This phase completes US-2 by building the `/auth` page that the user icon navigates to.

**Acceptance Criteria:**
- [ ] Route `/auth` renders auth choice page
- [ ] Page shows "Welcome!" heading
- [ ] Two buttons: "Sign In" and "Sign Up"
- [ ] "Sign In" button navigates to `/login`
- [ ] "Sign Up" button navigates to `/signup`
- [ ] Page is theme-aware and mobile responsive

**Independent Test Criteria:**
- [ ] `/auth` page accessible (no 404)
- [ ] "Welcome!" heading visible
- [ ] Both buttons render and are clickable
- [ ] Sign In button → navigates to `/login` (will 404 until US3)
- [ ] Sign Up button → navigates to `/signup` (will 404 until US4)
- [ ] Page works in light and dark mode
- [ ] Mobile responsive (buttons stack on small screens)

**Tasks:**

### 4.1: Auth Page Component

- [ ] T045 [P] [US2] Create my-website/src/pages/auth.tsx
- [ ] T046 [P] [US2] Import React, Layout from '@theme/Layout', Link from '@docusaurus/Link'
- [ ] T047 [P] [US2] Import styles from './auth.module.css'
- [ ] T048 [P] [US2] Create functional component AuthPage with Layout wrapper
- [ ] T049 [P] [US2] Set Layout title="Authentication" description="Login or sign up"
- [ ] T050 [P] [US2] Add main container with className="container"
- [ ] T051 [P] [US2] Add section with centered card layout (max-width: 500px)
- [ ] T052 [P] [US2] Add h1 heading: "Welcome!"
- [ ] T053 [P] [US2] Add subtitle: "Choose an option to continue"
- [ ] T054 [P] [US2] Add Link button to="/login" with className and text "🔐 Sign In"
- [ ] T055 [P] [US2] Add Link button to="/signup" with className and text "✨ Sign Up"
- [ ] T056 [P] [US2] Export AuthPage as default

### 4.2: Auth Page Styles

- [ ] T057 [P] [US2] Create my-website/src/pages/auth.module.css
- [ ] T058 [P] [US2] Define .authPage with padding and min-height
- [ ] T059 [P] [US2] Define .authCard with background: var(--ifm-card-background-color)
- [ ] T060 [P] [US2] Add .authCard with max-width: 500px, margin: auto, padding: 3rem
- [ ] T061 [P] [US2] Add .authCard with border-radius: 12px and box-shadow
- [ ] T062 [P] [US2] Define .authButton with full-width button styles
- [ ] T063 [P] [US2] Add .authButton hover state with transform: translateY(-2px)
- [ ] T064 [P] [US2] Add mobile breakpoint @media (max-width: 996px) with adjusted padding
- [ ] T065 [P] [US2] Add dark mode styles: [data-theme='dark'] .authCard adjustments

### 4.3: Verification

- [ ] T066 [US2] Run `npm run build` to verify page compiles
- [ ] T067 [US2] Run `npm run start` and navigate to http://localhost:3000/auth
- [ ] T068 [US2] Verify "Welcome!" heading displays
- [ ] T069 [US2] Verify both buttons render correctly
- [ ] T070 [US2] Click "Sign In" button - verify navigates to /login (404 expected)
- [ ] T071 [US2] Click "Sign Up" button - verify navigates to /signup (404 expected)
- [ ] T072 [US2] Test theme toggle: verify card background changes
- [ ] T073 [US2] Test mobile: resize to 320px, verify layout stacks vertically
- [ ] T074 [US2] Test from navbar: click user icon → should navigate to /auth successfully

**Files Created:**
- `my-website/src/pages/auth.tsx`
- `my-website/src/pages/auth.module.css`

**Verification Commands:**
```bash
cd my-website
npm run build
npm run start
# Navigate to http://localhost:3000/auth
# Click user icon in navbar → should land on auth page
```

**Story Status:** ✅ Complete and independently testable

---

## Phase 5: User Story 3 - Login Page

**User Story:**
> **As a** returning user
> **I want** a login page
> **So that** I can prepare for future authentication

**Acceptance Criteria:**
- [ ] Route `/login` renders login form
- [ ] Form has Email and Password fields
- [ ] Submit button labeled "Login"
- [ ] Notice: "Authentication coming soon"
- [ ] Client-side validation (email format, password required)
- [ ] Theme-aware and mobile responsive

**Independent Test Criteria:**
- [ ] `/login` page accessible (no 404)
- [ ] Email field (type: email, required)
- [ ] Password field (type: password, required, masked input)
- [ ] Submit button renders (disabled state)
- [ ] "Coming soon" notice displays
- [ ] Form validates on submit: empty fields → show errors
- [ ] Valid input → show "coming soon" message
- [ ] Works in light and dark mode
- [ ] Mobile responsive form layout

**Tasks:**

### 5.1: Login Page Component

- [ ] T075 [P] [US3] Create my-website/src/pages/login.tsx
- [ ] T076 [P] [US3] Import React, useState, FormEvent, Layout, styles
- [ ] T077 [P] [US3] Define LoginFormData interface: {email: string, password: string}
- [ ] T078 [P] [US3] Create functional component LoginPage with state management
- [ ] T079 [P] [US3] Add useState for formData, errors, isSubmitting
- [ ] T080 [P] [US3] Implement handleChange for input fields
- [ ] T081 [P] [US3] Implement validateForm function: email regex, password min 8 chars
- [ ] T082 [P] [US3] Implement handleSubmit: preventDefault, validate, show message if valid
- [ ] T083 [P] [US3] Add Layout wrapper with title="Login"
- [ ] T084 [P] [US3] Add form with onSubmit={handleSubmit}
- [ ] T085 [P] [US3] Add email input: type="email", required, onChange, value
- [ ] T086 [P] [US3] Add password input: type="password", required, onChange, value
- [ ] T087 [P] [US3] Add submit button with disabled={isSubmitting}
- [ ] T088 [P] [US3] Add info alert div with "Authentication coming soon" message
- [ ] T089 [P] [US3] Add error display for validation messages
- [ ] T090 [P] [US3] Export LoginPage as default

### 5.2: Login Page Styles

- [ ] T091 [P] [US3] Create my-website/src/pages/login.module.css
- [ ] T092 [P] [US3] Define .loginPage with padding and min-height
- [ ] T093 [P] [US3] Define .loginForm with max-width: 400px, margin: auto
- [ ] T094 [P] [US3] Define .formGroup with margin-bottom: 1.5rem
- [ ] T095 [P] [US3] Define label styles with font-weight: 600, color: var(--ifm-heading-color)
- [ ] T096 [P] [US3] Define .input with full-width, padding, border: var(--ifm-color-emphasis-300)
- [ ] T097 [P] [US3] Add .input focus state with border: var(--ifm-color-primary)
- [ ] T098 [P] [US3] Define .submitButton with button--primary styles
- [ ] T099 [P] [US3] Define .notice with info alert styling
- [ ] T100 [P] [US3] Add dark mode styles for inputs and notice
- [ ] T101 [P] [US3] Add mobile breakpoint @media (max-width: 996px)

### 5.3: Verification

- [ ] T102 [US3] Run `npm run build` to verify compilation
- [ ] T103 [US3] Run `npm run start` and navigate to http://localhost:3000/login
- [ ] T104 [US3] Verify email and password fields render
- [ ] T105 [US3] Test empty submit: verify error messages appear
- [ ] T106 [US3] Test invalid email: enter "notanemail" → verify email error
- [ ] T107 [US3] Test short password: enter "123" → verify password error
- [ ] T108 [US3] Test valid input: enter valid email + password → verify "coming soon" message
- [ ] T109 [US3] Test password masking: verify password field shows dots/asterisks
- [ ] T110 [US3] Test theme toggle: verify form works in both modes
- [ ] T111 [US3] Test mobile: resize to 320px, verify form is usable
- [ ] T112 [US3] Test keyboard: Tab through fields, Enter to submit

**Files Created:**
- `my-website/src/pages/login.tsx`
- `my-website/src/pages/login.module.css`

**Verification Commands:**
```bash
cd my-website
npm run build
npm run start
# Navigate to http://localhost:3000/login
# From /auth page, click "Sign In" → should land on login page
```

**Story Status:** ✅ Complete and independently testable

---

## Phase 6: User Story 4 - Signup Page

**User Story:**
> **As a** new user
> **I want** a signup page
> **So that** I can prepare for future account creation

**Acceptance Criteria:**
- [ ] Route `/signup` renders signup form
- [ ] Form has Name, Email, and Password fields
- [ ] Submit button labeled "Create Account"
- [ ] Notice: "Signup will be enabled soon"
- [ ] Client-side validation (all fields required, email format, password min length)
- [ ] Theme-aware and mobile responsive

**Independent Test Criteria:**
- [ ] `/signup` page accessible (no 404)
- [ ] Name field (type: text, required)
- [ ] Email field (type: email, required)
- [ ] Password field (type: password, required, masked)
- [ ] Submit button renders (disabled state)
- [ ] "Coming soon" notice displays
- [ ] Form validates: empty fields → errors, invalid format → errors
- [ ] Valid input → "coming soon" message
- [ ] Works in light and dark mode
- [ ] Mobile responsive

**Tasks:**

### 6.1: Signup Page Component

- [ ] T113 [P] [US4] Create my-website/src/pages/signup.tsx
- [ ] T114 [P] [US4] Import React, useState, FormEvent, Layout, styles
- [ ] T115 [P] [US4] Define SignupFormData interface: {name: string, email: string, password: string}
- [ ] T116 [P] [US4] Create functional component SignupPage with state management
- [ ] T117 [P] [US4] Add useState for formData, errors, isSubmitting
- [ ] T118 [P] [US4] Implement handleChange for all input fields
- [ ] T119 [P] [US4] Implement validateForm: name min 2 chars, email regex, password min 8 chars
- [ ] T120 [P] [US4] Implement handleSubmit: preventDefault, validate, show message if valid
- [ ] T121 [P] [US4] Add Layout wrapper with title="Sign Up"
- [ ] T122 [P] [US4] Add form with onSubmit={handleSubmit}
- [ ] T123 [P] [US4] Add name input: type="text", required, onChange, value
- [ ] T124 [P] [US4] Add email input: type="email", required, onChange, value
- [ ] T125 [P] [US4] Add password input: type="password", required, onChange, value
- [ ] T126 [P] [US4] Add submit button "Create Account" with disabled={isSubmitting}
- [ ] T127 [P] [US4] Add info alert with "Signup will be enabled soon" message
- [ ] T128 [P] [US4] Add error display for validation messages (per field)
- [ ] T129 [P] [US4] Export SignupPage as default

### 6.2: Signup Page Styles

- [ ] T130 [P] [US4] Create my-website/src/pages/signup.module.css
- [ ] T131 [P] [US4] Define .signupPage with padding and min-height
- [ ] T132 [P] [US4] Define .signupForm with max-width: 400px, margin: auto
- [ ] T133 [P] [US4] Define .formGroup with margin-bottom: 1.5rem
- [ ] T134 [P] [US4] Define label styles (same as login page)
- [ ] T135 [P] [US4] Define .input with full-width, padding, Infima variables
- [ ] T136 [P] [US4] Add .input focus state with primary color border
- [ ] T137 [P] [US4] Define .submitButton with button--primary class styles
- [ ] T138 [P] [US4] Define .notice with info alert styling
- [ ] T139 [P] [US4] Define .error with red text color for validation errors
- [ ] T140 [P] [US4] Add dark mode styles for all elements
- [ ] T141 [P] [US4] Add mobile breakpoint @media (max-width: 996px)

### 6.3: Verification

- [ ] T142 [US4] Run `npm run build` to verify compilation
- [ ] T143 [US4] Run `npm run start` and navigate to http://localhost:3000/signup
- [ ] T144 [US4] Verify name, email, password fields render
- [ ] T145 [US4] Test empty submit: verify all field errors appear
- [ ] T146 [US4] Test short name: enter "a" → verify name error (min 2 chars)
- [ ] T147 [US4] Test invalid email: enter "notanemail" → verify email error
- [ ] T148 [US4] Test short password: enter "123" → verify password error (min 8)
- [ ] T149 [US4] Test valid input: enter all valid → verify "coming soon" message
- [ ] T150 [US4] Test password masking: verify password shows dots/asterisks
- [ ] T151 [US4] Test theme toggle: verify form works in both modes
- [ ] T152 [US4] Test mobile: resize to 320px, verify form layout
- [ ] T153 [US4] Test keyboard navigation: Tab through all fields, Enter to submit

**Files Created:**
- `my-website/src/pages/signup.tsx`
- `my-website/src/pages/signup.module.css`

**Verification Commands:**
```bash
cd my-website
npm run build
npm run start
# Navigate to http://localhost:3000/signup
# From /auth page, click "Sign Up" → should land on signup page
```

**Story Status:** ✅ Complete and independently testable

---

## Phase 7: Integration Testing & Polish

**Goal**: Verify complete user flows and polish edge cases

**Integration Test Criteria:**
- [ ] Complete flow: User icon → /auth → /login → form validation
- [ ] Complete flow: User icon → /auth → /signup → form validation
- [ ] All pages accessible from navbar and direct URLs
- [ ] Theme persistence across all pages
- [ ] Mobile menu navigation works end-to-end
- [ ] Accessibility: full keyboard navigation through all flows

**Tasks:**

### 7.1: End-to-End Flow Testing

- [ ] T154 Test navbar user icon → /auth → /login flow
- [ ] T155 Test navbar user icon → /auth → /signup flow
- [ ] T156 Test direct URL access: /auth, /login, /signup all load
- [ ] T157 Test browser back button navigation through auth flows
- [ ] T158 Test theme toggle persistence: switch theme on /auth, verify /login inherits
- [ ] T159 Test mobile menu: open hamburger → click user icon → navigate to auth

### 7.2: Accessibility Testing

- [ ] T160 Test full keyboard navigation: Tab through navbar → user icon → auth pages
- [ ] T161 Verify all form labels have proper for attributes
- [ ] T162 Verify user icon has aria-label="User menu"
- [ ] T163 Test screen reader: verify all interactive elements announced
- [ ] T164 Verify focus indicators visible on all focusable elements
- [ ] T165 Test Enter key: activates buttons and submits forms

### 7.3: Cross-Browser Testing

- [ ] T166 Test in Chrome (latest)
- [ ] T167 Test in Firefox (latest)
- [ ] T168 Test in Safari (if available)
- [ ] T169 Test in Edge (latest)

### 7.4: Responsive Testing

- [ ] T170 Test at 320px width (small mobile)
- [ ] T171 Test at 768px width (tablet)
- [ ] T172 Test at 996px width (Docusaurus breakpoint)
- [ ] T173 Test at 1280px+ width (desktop)

### 7.5: Build & Deployment Validation

- [ ] T174 Run `npm run build` - verify no errors
- [ ] T175 Run `npm run serve` - test production build locally
- [ ] T176 Verify build output: check build/auth/, build/login/, build/signup/ directories exist
- [ ] T177 Verify static HTML generated for all auth pages
- [ ] T178 Check bundle size: auth pages should be <5KB each
- [ ] T179 Verify no console warnings or errors in production build

**Verification Commands:**
```bash
cd my-website
npm run clear  # Clear cache
npm run build  # Production build
npm run serve  # Test production build
# Test all flows in production mode
```

---

## Phase 8: Documentation & Cleanup

**Goal**: Update documentation and finalize implementation

**Tasks:**

- [ ] T180 Update README if navbar changes affect user documentation
- [ ] T181 Verify all task checkboxes in this file are completed
- [ ] T182 Run final `npm run build` to confirm production-ready
- [ ] T183 Commit changes with descriptive message (see plan.md for format)
- [ ] T184 Push branch to remote: `git push origin 004-navbar-auth-ui-cleanup`
- [ ] T185 Create pull request or merge to main (per project workflow)

**Verification:**
```bash
git status  # Verify all files committed
git log -1  # Verify commit message
```

---

## Dependency Graph

```
Phase 1 (Setup)
    ↓
Phase 2 (US-1: Navbar Cleanup) ← Can start immediately
    ↓
Phase 3 (US-2: User Icon) ← Depends on Phase 2 (navbar config)
    ↓
Phase 4 (US-2: Auth Page) ← Depends on Phase 3 (user icon navigation)
    ↓
Phase 5 (US-3: Login Page) ← Depends on Phase 4 (auth page links to login)
    ↓
Phase 6 (US-4: Signup Page) ← Depends on Phase 4 (auth page links to signup)
    ↓
Phase 7 (Integration Testing) ← Depends on all user stories complete
    ↓
Phase 8 (Documentation) ← Final phase
```

**Critical Path:** Setup → US-1 → US-2 (Icon) → US-2 (Auth Page) → US-3/US-4 (parallel) → Integration → Documentation

**Parallel Opportunities:**
- Phase 5 (US-3: Login) and Phase 6 (US-4: Signup) can be implemented in parallel after Phase 4 completes
- Within each phase, tasks marked with [P] can be done concurrently

---

## Execution Strategy

### MVP Scope (Minimum Viable Product)
For fastest delivery to production:
1. **Phase 1**: Setup
2. **Phase 2**: US-1 (Navbar Cleanup)
3. **Phase 3**: US-2 (User Icon)
4. **Phase 4**: US-2 (Auth Page)

**Result:** Clean navbar with auth entry point (forms come later)

### Full Feature Scope
Complete all phases 1-8 for full functionality.

### Incremental Delivery
- **Sprint 1**: Phases 1-2 (navbar cleanup) → Deploy
- **Sprint 2**: Phases 3-4 (user icon + auth page) → Deploy
- **Sprint 3**: Phases 5-6 (login + signup forms) → Deploy
- **Sprint 4**: Phases 7-8 (testing + polish) → Final deploy

---

## Task Execution Tips

1. **Work Sequentially Within Phases**: Complete tasks in order within each phase
2. **Test After Each Phase**: Run build and manual tests before moving to next phase
3. **Use Parallel Tasks**: Tasks marked [P] within a phase can be done simultaneously
4. **Verify Baseline**: Always run `npm run build` before starting to establish working state
5. **Commit Frequently**: Commit after each completed phase for rollback safety
6. **Follow File Paths**: Task descriptions include exact file paths - use them precisely

---

## Verification Checklist

After completing all tasks:

- [ ] All user stories have acceptance criteria met
- [ ] `npm run build` succeeds without errors
- [ ] No TypeScript compilation errors
- [ ] No React warnings in console
- [ ] All auth pages accessible (/auth, /login, /signup)
- [ ] Navbar shows only Logo + "Learning" on left
- [ ] User icon visible and functional in navbar
- [ ] Mobile responsive (tested at 320px, 768px, 996px)
- [ ] Theme toggle works on all pages (light/dark)
- [ ] Keyboard navigation functional
- [ ] ARIA labels present
- [ ] Form validation works client-side
- [ ] "Coming soon" notices display on forms

---

## Success Metrics

**Task Completion:** 185 tasks total
- Phase 1 (Setup): 6 tasks
- Phase 2 (US-1): 9 tasks
- Phase 3 (US-2 Icon): 29 tasks
- Phase 4 (US-2 Auth Page): 30 tasks
- Phase 5 (US-3 Login): 38 tasks
- Phase 6 (US-4 Signup): 41 tasks
- Phase 7 (Integration): 26 tasks
- Phase 8 (Documentation): 6 tasks

**Estimated Time:**
- MVP (Phases 1-4): 1.5 hours
- Full Feature (Phases 1-8): 3-4 hours

**Quality Gates:**
- Build passes: REQUIRED after each phase
- Manual testing: REQUIRED after phases 2, 3, 4, 5, 6
- Integration testing: REQUIRED in phase 7
- Production build: REQUIRED in phase 8

---

## Notes

- **No Backend**: All forms are client-side only (no API calls)
- **No New Dependencies**: Uses only existing Docusaurus packages
- **Mobile First**: CSS is mobile-first (base = mobile, media queries = desktop)
- **Theme Aware**: All styles use Infima CSS variables for automatic theme support
- **Accessibility**: WCAG AA compliance required for all interactive elements

---

**Generated:** 2025-12-17
**Ready for:** `/sp.implement` (implementation execution)
**Branch:** `004-navbar-auth-ui-cleanup`
**Status:** ✅ Tasks Complete - Ready for Implementation
